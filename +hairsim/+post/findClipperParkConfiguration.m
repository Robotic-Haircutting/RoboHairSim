function [qPark, report] = findClipperParkConfiguration(baseCtx, tool, headQuery, stationsDeg, options)
%FINDCLIPPERPARKCONFIGURATION A clipper-safe "Home" for the camClipper
%   pipeline. The GUI Home pose (createGuiHomeConfiguration) was tuned for
%   the old sphere: with the real clipper it leaves the blade 0.3 mm INSIDE
%   the head at theta = 180 deg. Cross-station transits park the arm here
%   while the rail moves, so it must be clear at EVERY station.
%
%   From Home, the TCP is pushed straight away from the head centre (same
%   orientation) in StepM steps, each by IK seeded with the previous q,
%   until the clipper clears the head by ParkClearanceM at all stationsDeg
%   (exact clipper model, robot links vs head + environment, arm + clipper
%   self-collision). If no lift clears every station, the lift valid at
%   the MOST stations is returned and report.perStation says which: the
%   caller drops the others from its station list.
%   report: liftM, perStation (thetaDeg, clearanceM, ok), validStationsDeg.

arguments
    baseCtx (1, 1) struct
    tool (1, 1) struct
    headQuery (1, 1) struct
    stationsDeg (1, :) double
    options.ParkClearanceM (1, 1) double {mustBePositive} = 0.02
    options.StepM (1, 1) double {mustBePositive} = 0.01
    options.MaxLiftM (1, 1) double {mustBePositive} = 0.15
end

params = struct('ikWeights', [0.01, 0.01, 0.01, 5, 5, 5], 'maxToolTipErrorM', 0.002, ...
    'collisionMarginM', 0.001, 'maxJointStepRad', deg2rad(20));
contexts = arrayfun(@(th) hairsim.post.createStationContext(baseCtx, deg2rad(th), tool), stationsDeg);
ref = contexts(1);
qHome = baseCtx.qHome(:);
T0 = getTransform(ref.robot, qHome, char(ref.endEffector)) * tool.T_ee_tcp;
d = T0(1:3, 4)' - baseCtx.headCenterWorldM;
d = d / norm(d);

qPrev = qHome;
best = struct('count', -1, 'q', [], 'lift', nan, 'perStation', []);
for lift = 0:options.StepM:options.MaxLiftM
    if lift == 0
        q = qHome;
    else
        T = T0;
        T(1:3, 4) = T0(1:3, 4) + lift * d';
        q = hairsim.post.solveClipperPose(ref, params, qPrev, T, tool, headQuery, "fast", true);
        if isempty(q)
            continue;
        end
    end
    [ok, perStation] = validAtAll(q, contexts, stationsDeg, tool, headQuery, options.ParkClearanceM, baseCtx);
    count = nnz([perStation.ok]);
    if count > best.count
        best = struct('count', count, 'q', q, 'lift', lift, 'perStation', perStation);
    end
    if ok
        break;
    end
    qPrev = q;
end
if best.count <= 0
    error("hairsim:post:NoParkConfiguration", ...
        "No clipper-safe park configuration within %.0f mm of Home.", 1000 * options.MaxLiftM);
end
qPark = best.q;
report = struct('liftM', best.lift, 'perStation', best.perStation, ...
    'validStationsDeg', stationsDeg([best.perStation.ok]));
end

function [ok, perStation] = validAtAll(q, contexts, stationsDeg, tool, headQuery, clearanceM, baseCtx)
perStation = struct('thetaDeg', num2cell(stationsDeg), 'clearanceM', nan, 'ok', false);
for i = 1:numel(contexts)
    s = contexts(i);
    T = getTransform(s.robot, q, char(s.endEffector)) * tool.T_ee_tcp;
    c = hairsim.tool.evaluateClipperClearance(T, tool, headQuery, 'CapM', 0.2);
    robotClear = hairsim.motion.validateClipperRobotCollision(s.robot, q, s.endEffector, ...
        s.toolMount, s.headCollision, 0.001, true, 'SkipClipperHeadCheck', false);
    selfFree = ~checkCollision(s.armWithToolRobot, q, 'SkippedSelfCollisions', 'parent');
    perStation(i).clearanceM = c.minM;
    perStation(i).ok = c.minM >= clearanceM && robotClear && selfFree;
end
ok = all([perStation.ok]);
end
