function [teleopState, poseCommand] = stepTeleop(teleopState, robotAppState, cutEnabled)
%STEPTELEOP One clipper-Teleoperation tick: queued input -> new surface
%   state -> TCP pose -> IK + safety -> proposed PoseCommand ([] when there
%   is nothing to do).
%     1. input moves the foot over the scalp (tangent step, projected back
%        onto the mesh; a projection that jumps more than 2 steps -- ear,
%        neck -- is refused), turns / snaps the feed, lifts / lowers, rolls;
%     2. target pose = hairsim.tool.buildTcpPose at the standoff, lifted by
%        hairsim.tool.resolveContactStandoff when the blade would touch
%        the head (as every CAM pose is);
%     3. the move from the current pose is limited to MaxTranslationPerTickM
%        / MaxRotationPerTickRad; while limited, later ticks keep moving
%        ("settling") without new input;
%     4. hairsim.post.solveClipperPose (fast, then extended seeds): IK,
%        robot vs head, joint step, clipper vs exact head mesh, self
%        collision -- the post-processor's gates.
%   A refused move leaves the robot and the surface state where they were
%   and reports the reason (PoseCommand.solved = false).
%   With a region selected (hairsim.app.teleop.setRegion) a foot move that
%   would leave it is refused (OUTSIDE_REGION) -- unless the foot is already
%   outside and the move brings it closer, so the clipper can be driven in.
%
%   Rail slide (C / V, rail setups only): the base moves RailStepRad along
%   the ring, within RailLimitsRad, and the arm re-solves so the clipper
%   stays where it is (same IK + safety gates at the new station). The
%   PoseCommand carries railThetaRad; hairsim.app.robot.applyPoseCommand
%   moves the robot model there. A slide is a tick of its own: other keys
%   queued with it run on the next tick.
%
%   Cutting: cutFlag only on an accepted tick that PUSHES FORWARD (W/Up)
%   with Space held and cutEnabled -- the comb lifts hair it is pushed
%   into; pulling back, sliding or turning never cuts. Whether the strand
%   is then cut is the comb-lift cut model's call (same predicate as Auto:
%   blade close enough, root in the footprint, feed against the flow).

arguments
    teleopState (1, 1) struct
    robotAppState (1, 1) struct
    cutEnabled (1, 1) logical
end

poseCommand = [];
if any(~isfinite(teleopState.surface.footWorldM))
    error("hairsim:TeleopNotSynced", "Call hairsim.app.teleop.syncToRobot before stepTeleop.");
end
input = teleopState.pending;
hasInput = input.feed ~= 0 || input.side ~= 0 || input.yaw ~= 0 || input.standoff ~= 0 || ...
    input.roll ~= 0 || input.rail ~= 0 || input.snapToFlow;
if ~hasInput && ~teleopState.settling
    return;
end
teleopState.pending = hairsim.app.teleop.emptyInput();
tool = teleopState.tool;
query = teleopState.headQuery;
mc = robotAppState.modelContext;
qCurrent = robotAppState.state.qCurrent(:);
T_current = getTransform(mc.robot, qCurrent, char(mc.endEffector)) * tool.T_ee_tcp;

if input.rail ~= 0
    rest = input;
    rest.rail = 0;
    teleopState.pending = rest;
    [teleopState, poseCommand] = slideRail(teleopState, robotAppState, input.rail, T_current);
    return;
end

% ---- 1. new surface state
s = teleopState.surface;
n = hairsim.cam.smoothedNormalAt(s.footWorldM, query);
x = tangent(s.feedWorld, n);
if input.snapToFlow && ~isempty(teleopState.hairFlow)
    x = tangent(-flowAt(teleopState, s.footWorldM), n);
end
if input.yaw ~= 0
    a = input.yaw * teleopState.yawStepRad;
    x = x * cos(a) + cross(n, x) * sin(a);
end
if input.feed ~= 0 || input.side ~= 0
    y = cross(-n, x);                                  % TCP y: right of the feed, seen from outside
    intended = s.footWorldM + teleopState.moveStepM * (input.feed * x + input.side * y);
    [~, foot] = hairsim.tool.signedDistanceToHead(intended, query, 0.05);
    if any(~isfinite(foot)) || norm(foot - s.footWorldM) > 2 * teleopState.moveStepM * ...
            max(1, hypot(input.feed, input.side))
        [teleopState, poseCommand] = refuse(teleopState, robotAppState, "FOOT_JUMP");
        return;
    end
    [inside, distanceM] = hairsim.app.teleop.regionMembership(teleopState, [s.footWorldM; foot]);
    if ~inside(2) && ~(distanceM(2) < distanceM(1))
        [teleopState, poseCommand] = refuse(teleopState, robotAppState, "OUTSIDE_REGION");
        return;
    end
    n = hairsim.cam.smoothedNormalAt(foot, query);
    x = tangent(x, n);
    s.footWorldM = foot;
end
s.feedWorld = x;
s.standoffM = min(max(s.standoffM + input.standoff * teleopState.standoffStepM, tool.nominalStandoffM), ...
    teleopState.maxStandoffM);
s.rollRad = min(max(s.rollRad + input.roll * teleopState.rollStepRad, -teleopState.maxRollRad), ...
    teleopState.maxRollRad);

% ---- 2. target pose (blade-edge drop of a rolled clipper as in hairsim.cam.buildSegments)
T_nominal = hairsim.tool.buildTcpPose(s.footWorldM, n, x, tool, ...
    'StandoffM', s.standoffM + tool.bladeHalfWidthM * abs(sin(s.rollRad)), 'RollRad', s.rollRad);
[T_target, canSit] = hairsim.tool.resolveContactStandoff(T_nominal, n, tool, query);
if ~canSit
    [teleopState, poseCommand] = refuse(teleopState, robotAppState, "CLIPPER_CANNOT_SIT");
    return;
end

% ---- 3. per-tick motion limit
[T_command, limited] = limitStep(T_current, T_target, teleopState.maxTranslationPerTickM, ...
    teleopState.maxRotationPerTickRad);

% ---- 4. IK + safety gates
sctx = stationContext();
[q, T_actual, rejected] = hairsim.post.solveClipperPose(sctx, teleopState.params, qCurrent, T_command, ...
    tool, query, "fast", true);
if isempty(q)
    [q, T_actual, rejected] = hairsim.post.solveClipperPose(sctx, teleopState.params, qCurrent, T_command, ...
        tool, query, "extended", true);
end
if isempty(q)
    [teleopState, poseCommand] = refuse(teleopState, robotAppState, upper(dominantReason(rejected)));
    return;
end
teleopState.surface = s;
teleopState.settling = limited;
pushing = input.feed > 0 && input.side == 0;
cutFlag = cutEnabled && teleopState.spaceHeld && pushing;

status = "ACCEPTED";
if limited
    status = "ACCEPTED:SETTLING";
end
teleopState.hud = hud(teleopState, status, T_actual, s);
T_ee = T_actual / tool.T_ee_tcp;
tip = hairsim.getMountedClipperPoint(T_ee, mc.toolMount.T_ee_tool);
targetTip = hairsim.getMountedClipperPoint(T_target / tool.T_ee_tcp, mc.toolMount.T_ee_tool);
commandTip = hairsim.getMountedClipperPoint(T_command / tool.T_ee_tcp, mc.toolMount.T_ee_tool);
poseCommand = struct('qCandidate', q(:), 'solved', true, 'toolTipWorldM', tip, ...
    'statusLabel', status, 'toolTipErrorM', norm(T_actual(1:3, 4) - T_command(1:3, 4)), ...
    'targetTipWorldM', targetTip, 'desiredTipWorldM', targetTip, 'safeTipWorldM', commandTip, ...
    'governorActive', limited, 'cutFlag', cutFlag);

    function sctx = stationContext()
        % the camClipper post-processor's robot context at the current base,
        % rebuilt only when the robot model changes (base switch, rail slide)
        if isempty(teleopState.station.robot) || teleopState.station.robot ~= mc.robot
            teleopState.station = struct('robot', mc.robot, ...
                'sctx', hairsim.post.createStationContext(baseContext(teleopState, mc), NaN, tool));
        end
        sctx = teleopState.station.sctx;
    end
end

% =========================================================================

function [teleopState, poseCommand] = slideRail(teleopState, robotAppState, steps, T_current)
% move the base along the rail, the clipper held at T_current
mc = robotAppState.modelContext;
tool = teleopState.tool;
qCurrent = robotAppState.state.qCurrent(:);
if ~isfield(mc, 'railContextBuilder')
    [teleopState, poseCommand] = refuse(teleopState, robotAppState, "NO_RAIL");
    return;
end
theta = robotAppState.state.railThetaRad;
newTheta = min(max(theta + steps * teleopState.railStepRad, teleopState.railLimitsRad(1)), ...
    teleopState.railLimitsRad(2));
if abs(newTheta - theta) < 1e-9
    [teleopState, poseCommand] = refuse(teleopState, robotAppState, "RAIL_LIMIT");
    return;
end
baseCtx = baseContext(teleopState, mc);
baseCtx.railContextBuilder = mc.railContextBuilder;
sctx = hairsim.post.createStationContext(baseCtx, newTheta, tool);
[q, T_actual, rejected] = hairsim.post.solveClipperPose(sctx, teleopState.params, qCurrent, T_current, ...
    tool, teleopState.headQuery, "fast", true);
if isempty(q)
    [q, T_actual, rejected] = hairsim.post.solveClipperPose(sctx, teleopState.params, qCurrent, T_current, ...
        tool, teleopState.headQuery, "extended", true);
end
if isempty(q)
    [teleopState, poseCommand] = refuse(teleopState, robotAppState, "RAIL_" + upper(dominantReason(rejected)));
    return;
end
teleopState.settling = false;
status = sprintf("ACCEPTED:RAIL %.0f deg", rad2deg(newTheta));
teleopState.hud = hud(teleopState, status, T_actual, teleopState.surface);
tip = hairsim.getMountedClipperPoint(T_actual / tool.T_ee_tcp, mc.toolMount.T_ee_tool);
poseCommand = struct('qCandidate', q(:), 'solved', true, 'toolTipWorldM', tip, ...
    'statusLabel', status, 'toolTipErrorM', norm(T_actual(1:3, 4) - T_current(1:3, 4)), ...
    'targetTipWorldM', tip, 'desiredTipWorldM', tip, 'safeTipWorldM', tip, ...
    'governorActive', false, 'cutFlag', false, 'railThetaRad', newTheta);
end

function baseCtx = baseContext(teleopState, mc)
n = numel(mc.qHome);
baseCtx = struct('robot', mc.robot, 'endEffector', mc.endEffector, 'toolMount', mc.toolMount, ...
    'headCollision', {mc.headCollision}, 'qHome', mc.qHome(:), ...
    'headCenterWorldM', teleopState.headCenterWorldM, ...
    'redundantJointIndices', 1:n, 'numJoints', n);
end

function x = tangent(v, n)
x = v - dot(v, n) * n;
x = x / norm(x);
end

function f = flowAt(teleopState, point)
[~, k] = min(vecnorm(teleopState.hairRoots - point, 2, 2));
f = teleopState.hairFlow(k, :);
end

function [T, limited] = limitStep(T0, T1, maxTranslationM, maxRotationRad)
% move from T0 toward T1 by at most the given translation / rotation
d = T1(1:3, 4) - T0(1:3, 4);
dR = T0(1:3, 1:3)' * T1(1:3, 1:3);
angle = acos(min(1, max(-1, (trace(dR) - 1) / 2)));
a = min([1, maxTranslationM / max(norm(d), eps), maxRotationRad / max(angle, eps)]);
limited = a < 1;
T = eye(4);
T(1:3, 4) = T0(1:3, 4) + a * d;
if angle < 1e-9
    T(1:3, 1:3) = T1(1:3, 1:3);
    return;
end
axis = [dR(3, 2) - dR(2, 3); dR(1, 3) - dR(3, 1); dR(2, 1) - dR(1, 2)];
if norm(axis) < 1e-9          % ~180 deg: any axis perpendicular to the rotation's fixed plane
    [V, D] = eig(dR);
    [~, i] = min(abs(diag(D) - 1));
    axis = real(V(:, i));
end
axis = axis / norm(axis);
K = [0, -axis(3), axis(2); axis(3), 0, -axis(1); -axis(2), axis(1), 0];
th = a * angle;
T(1:3, 1:3) = T0(1:3, 1:3) * (eye(3) + sin(th) * K + (1 - cos(th)) * (K * K));
end

function reason = dominantReason(rejected)
% the gate that rejected the most candidates (a candidate counts at the
% first gate it fails); none at all = the solver found nothing
names = string(fieldnames(rejected))';
counts = cellfun(@(f) rejected.(f), cellstr(names));
reason = "notSolved";
if any(counts > 0)
    [~, i] = max(counts);
    reason = names(i);
end
end

function [teleopState, poseCommand] = refuse(teleopState, robotAppState, reason)
teleopState.settling = false;
teleopState.hud.status = "REJECTED:" + reason;
mc = robotAppState.modelContext;
q = robotAppState.state.qCurrent(:);
tip = hairsim.getMountedClipperPoint(getTransform(mc.robot, q, char(mc.endEffector)), mc.toolMount.T_ee_tool);
poseCommand = struct('qCandidate', q, 'solved', false, 'toolTipWorldM', tip, ...
    'statusLabel', "REJECTED:" + reason, 'toolTipErrorM', 0, 'targetTipWorldM', tip, ...
    'desiredTipWorldM', tip, 'safeTipWorldM', tip, 'governorActive', false, 'cutFlag', false);
end

function h = hud(teleopState, status, T_actual, s)
tool = teleopState.tool;
query = teleopState.headQuery;
[height, ~] = hairsim.tool.signedDistanceToHead(T_actual(1:3, 4)', query, 0.2);
clearance = hairsim.tool.evaluateClipperClearance(T_actual, tool, query, 'CapM', 0.2);
[inRegion, regionDistanceM] = hairsim.app.teleop.regionMembership(teleopState, s.footWorldM);
feedAngle = NaN;
if ~isempty(teleopState.hairFlow)
    against = -flowAt(teleopState, s.footWorldM);
    feedAngle = acosd(max(-1, min(1, dot(T_actual(1:3, 1)', against) / norm(against))));
end
h = struct('status', status, 'standoffMm', 1000 * height, ...
    'inContact', height <= tool.maxLocalHeightM, 'feedAngleDeg', feedAngle, ...
    'clearanceMm', 1000 * clearance.minM, 'rollDeg', rad2deg(s.rollRad), ...
    'inRegion', inRegion, 'regionDistanceMm', 1000 * regionDistanceM);
end
