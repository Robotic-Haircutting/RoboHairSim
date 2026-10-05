function result = planSafeConfigurationTransition(qa, qb, ctx, options)
%PLANSAFECONFIGURATIONTRANSITION Collision-free connection between two legal joint configurations.
%   No region / lane / rail semantics: only two joint configurations, the robot with its clipper
%   tool, and the collision objects. Structure (symmetric):
%       qa -> qa_clear -> qb_clear -> qb
%   qa_clear (qb_clear) is reached by RETREATING the clipper away from the head, keeping the tool
%   orientation, in StepM steps until the clipper sphere is ClearM from the head mesh. The direction of
%   every step is (sphere centre - nearest head-mesh point), re-evaluated at each step. The approach to
%   qb is the retreat path from qb, reversed. The middle qa_clear -> qb_clear is tried in this order:
%     1. straight line in joint space (<= JointStepDeg per sample);
%     2. a head-centred shell arc (fallback candidate, options.EnableShell): the tool tip moves radially
%        to a shell of radius rho = max(|pa - c_H|, |pb - c_H|) + ShellMarginM, follows a spherical arc
%        (slerp of the direction from the head centre c_H; the tool orientation is slerped) and comes back
%        radially to qb_clear; every sample by IK, then a short joint blend onto qb_clear.
%   NOTHING is assumed safe. Every sample of every segment goes through the planner's own path gates:
%   clipper sphere >= 1 mm from the head (safety mesh), robot (clipper tool excluded, as the planner)
%   vs the collision objects with a 1 mm margin, no arm self collision, joint step limit, finite q.
%   Retreat / shell IK steps are seeded ONLY with the previous q (no Home, no redundant seeds) and
%   additionally limited to MaxRetreatDqRad per step, so the IK cannot jump to another solution branch.
%
%   Clearance policy (deterministic): options.ClearLadderM (default [0.060 0.100 0.150] m) is tried in order
%   and the FIRST rung that yields a feasible connection is returned ("smallest feasible retreat
%   distance"). A larger clearance is not automatically better: it moves the retreat points away from the
%   head but towards the arm's reach limit. options.ClearM (a single value) overrides the ladder.
%   result.clearM is the rung used (the last rung tried if none worked), result.ladderReasons the outcome
%   of every rung tried.
%
%   ctx fields: robot (with the clipper tool), armOnlyRobot, endEffector, toolMount,
%   headMeshSafetyWorld, headCenterWorldM, collisionObjects (cell: head pieces + environment).
%
%   result: feasible, reason ("OK" or START_RETREAT_IK, START_RETREAT_COLLISION, GOAL_APPROACH_IK,
%   GOAL_APPROACH_COLLISION, JOINT_TRANSFER_COLLISION, SHELL_TRANSFER_IK, SHELL_TRANSFER_COLLISION,
%   INVALID_ENDPOINT_A/B), detail, method ("joint" | "shell" | ""), jointTransferReason (why step 1
%   failed when the shell was used or also failed), Q (numel(qa) x N joint path qa -> qb), phase (N x 1:
%   RETREAT / TRANSFER / APPROACH), numRetreatStart, numTransfer, numApproach, planningTimeS (of the last rung),
%   clearM, ladderReasons.

arguments
    qa (:, 1) double {mustBeFinite}
    qb (:, 1) double {mustBeFinite}
    ctx (1, 1) struct
    options.ClearM (1, 1) double = nan
    options.ClearLadderM (1, :) double {mustBePositive} = [0.060, 0.100, 0.150]
    options.StepM (1, 1) double {mustBePositive} = 0.005
    options.MaxRetreatM (1, 1) double {mustBePositive} = 0.20
    options.MaxRetreatDqRad (1, 1) double {mustBePositive} = deg2rad(10)
    options.MaxJointStepRad (1, 1) double {mustBePositive} = deg2rad(60)
    options.JointStepDeg (1, 1) double {mustBePositive} = 5
    options.ShellMarginM (1, 1) double {mustBeNonnegative} = 0.010
    options.EnableShell (1, 1) logical = true
end

ladder = options.ClearLadderM;
if ~isnan(options.ClearM)
    ladder = options.ClearM;
end
optStruct = options;
ladderReasons = strings(0, 1);
for rung = 1:numel(ladder)
    optStruct.ClearM = ladder(rung);
    result = planAtClearance(qa, qb, ctx, optStruct);
    ladderReasons(end + 1, 1) = sprintf("%.0f mm: %s", 1000 * ladder(rung), result.reason); %#ok<AGROW>
    if result.feasible || startsWith(result.reason, "INVALID_ENDPOINT")
        break;
    end
end
result.clearM = optStruct.ClearM;
result.ladderReasons = ladderReasons;
end

function result = planAtClearance(qa, qb, ctx, options)
%PLANATCLEARANCE One connection attempt at the fixed clearance options.ClearM (struct of the options).
timer = tic;
gp = createGateParams(options);
numJoints = numel(qa);
result = struct('feasible', false, 'reason', "", 'detail', "", 'method', "", 'jointTransferReason', "", ...
    'Q', zeros(numJoints, 0), 'phase', strings(0, 1), 'numRetreatStart', 0, 'numTransfer', 0, 'numApproach', 0, ...
    'qaClear', nan(numJoints, 1), 'qbClear', nan(numJoints, 1), 'QPartial', zeros(numJoints, 0), 'planningTimeS', 0);

[okA, whyA] = gates(qa, [], ctx, gp);
if ~okA
    result = fail(result, "INVALID_ENDPOINT_A", whyA, timer); return;
end
[okB, whyB] = gates(qb, [], ctx, gp);
if ~okB
    result = fail(result, "INVALID_ENDPOINT_B", whyB, timer); return;
end

% ---- retreat from both ends
[QA, reasonA, detailA] = retreatPath(qa, ctx, gp);
if reasonA ~= "OK"
    result = fail(result, "START_RETREAT_" + reasonA, detailA, timer); return;
end
[QB, reasonB, detailB] = retreatPath(qb, ctx, gp);
if reasonB ~= "OK"
    result = fail(result, "GOAL_APPROACH_" + reasonB, detailB, timer); return;
end
qaClear = QA(:, end);
qbClear = QB(:, end);
result.qaClear = qaClear;
result.qbClear = qbClear;

% ---- middle: joint line first, shell arc as the fallback candidate
[Qmid, okMid, whyMid] = jointLine(qaClear, qbClear, ctx, gp);
method = "joint";
if ~okMid
    result.jointTransferReason = "JOINT_TRANSFER_COLLISION: " + whyMid;
    if ~options.EnableShell
        result = fail(result, "JOINT_TRANSFER_COLLISION", whyMid, timer); return;
    end
    [Qmid, reasonMid, detailMid, QPart] = shellArc(qaClear, qbClear, ctx, gp);
    method = "shell";
    if reasonMid ~= "OK"
        result.QPartial = QPart;
        result = fail(result, "SHELL_TRANSFER_" + reasonMid, detailMid, timer); return;
    end
end

Q = [QA, Qmid, fliplr(QB(:, 1:end - 1))];
result.Q = Q;
result.phase = [repmat("RETREAT", size(QA, 2), 1); repmat("TRANSFER", size(Qmid, 2), 1); ...
    repmat("APPROACH", size(QB, 2) - 1, 1)];
result.numRetreatStart = size(QA, 2);
result.numTransfer = size(Qmid, 2);
result.numApproach = size(QB, 2) - 1;
result.method = method;
result.feasible = true;
result.reason = "OK";
result.planningTimeS = toc(timer);
end

% ------------------------------------------------------------------------------------------------

function gp = createGateParams(options)
gp = struct('clearM', options.ClearM, 'stepM', options.StepM, 'maxRetreatM', options.MaxRetreatM, ...
    'maxRetreatDqRad', options.MaxRetreatDqRad, 'maxJointStepRad', options.MaxJointStepRad, ...
    'jointStepRad', deg2rad(options.JointStepDeg), 'shellMarginM', options.ShellMarginM, ...
    'dSafeM', 0.001, 'collisionMarginM', 0.001, 'maxToolTipErrorM', 0.005, ...
    'ikWeights', [0.01, 0.01, 0.01, 5, 5, 5]);
end

function result = fail(result, reason, detail, timer)
result.reason = reason;
result.detail = string(detail);
result.planningTimeS = toc(timer);
end

function [ok, why] = gates(q, qPrev, ctx, gp)
%GATES The planner's path gates for one configuration (see planAutonomousRegionHaircut/validatePlannedPath).
ok = false; why = "";
if ~all(isfinite(q))
    why = "non-finite joint values"; return;
end
T = getTransform(ctx.robot, q, char(ctx.endEffector));
center = hairsim.getMountedClipperPoint(T, ctx.toolMount.T_ee_tool_center);
clearance = hairsim.evaluateClipperSphereMeshClearance(center, ctx.headMeshSafetyWorld, ctx.toolMount.toolClearanceRadiusM);
if clearance.surfaceClearanceM < gp.dSafeM
    why = sprintf("clipper sphere %.1f mm from the head", 1000 * clearance.surfaceClearanceM); return;
end
if ~hairsim.motion.validateClipperRobotCollision(ctx.robot, q, ctx.endEffector, ctx.toolMount, ...
        ctx.collisionObjects, gp.collisionMarginM, true)
    why = "robot vs head/environment"; return;
end
if checkCollision(ctx.armOnlyRobot, q, 'SkippedSelfCollisions', 'parent')
    why = "arm self collision"; return;
end
if ~isempty(qPrev) && max(abs(q - qPrev)) > gp.maxJointStepRad + 1e-12
    why = sprintf("joint step %.1f deg", rad2deg(max(abs(q - qPrev)))); return;
end
ok = true;
end

function [q, reason, detail] = solveStep(qPrev, tipTargetM, R, ctx, gp)
%SOLVESTEP One free-space IK step seeded only by qPrev. reason: "" (ok), "IK", "COLLISION".
q = []; reason = "IK"; detail = "";
ik = inverseKinematics('RigidBodyTree', ctx.robot);
ik.SolverParameters.AllowRandomRestart = false;
[~, ~, ~, ~, log] = hairsim.motion.solveRobotForClipperTip( ...
    ctx.robot, ik, ctx.endEffector, gp.ikWeights, qPrev, tipTargetM, ctx.headCenterWorldM, ...
    ctx.toolMount.T_ee_tool, gp.maxToolTipErrorM, ...
    'RollAnglesRad', 0, 'BaseRotationWorldEe', R, ...
    'IncludeHomeSeed', false, 'IncludeInitialSeed', false, 'IncludeRedundantSeeds', false, ...
    'RandomSeedCount', 0, 'RedundantJointIndices', 1:numel(qPrev), 'InitialSeedConfiguration', qPrev, ...
    'HeadCollisionForIk', ctx.collisionObjects, 'ToolMountForCollision', ctx.toolMount, ...
    'CollisionMarginMForIk', gp.collisionMarginM, 'SkipClipperHeadCheckForIk', false);
bestStep = inf; nSolved = 0; why = "";
for i = 1:numel(log)
    if ~log(i).solved, continue; end
    nSolved = nSolved + 1;
    cand = log(i).q(:);
    dq = max(abs(cand - qPrev));
    if dq > gp.maxRetreatDqRad
        why = sprintf("IK step %.1f deg > %.1f deg limit (no branch jumps)", rad2deg(dq), rad2deg(gp.maxRetreatDqRad));
        continue;
    end
    if ~log(i).collisionClear
        why = "robot vs head/environment (IK candidate)"; continue;
    end
    [ok, w] = gates(cand, qPrev, ctx, gp);
    if ~ok, why = w; continue; end
    if dq < bestStep
        bestStep = dq; q = cand;
    end
end
if ~isempty(q)
    reason = "";
elseif nSolved > 0
    reason = "COLLISION"; detail = why;
else
    reason = "IK"; detail = "no IK solution within tolerance";
end
end

function [Q, reason, detail] = retreatPath(q0, ctx, gp)
%RETREATPATH Move the clipper away from the head (orientation held) until the sphere is gp.clearM from it.
Q = q0; reason = "OK"; detail = "";
T = getTransform(ctx.robot, q0, char(ctx.endEffector));
R = T(1:3, 1:3);
qPrev = q0;
maxSteps = ceil(gp.maxRetreatM / gp.stepM);
for step = 1:maxSteps
    T = getTransform(ctx.robot, qPrev, char(ctx.endEffector));
    center = hairsim.getMountedClipperPoint(T, ctx.toolMount.T_ee_tool_center);
    tip = hairsim.getMountedClipperPoint(T, ctx.toolMount.T_ee_tool);
    clearance = hairsim.evaluateClipperSphereMeshClearance(center, ctx.headMeshSafetyWorld, ctx.toolMount.toolClearanceRadiusM);
    if clearance.surfaceClearanceM >= gp.clearM
        return;
    end
    [closest, ~, dist] = hairsim.closestPointOnMesh(center, ctx.headMeshSafetyWorld);
    if dist < 1e-9
        reason = "COLLISION"; detail = "sphere centre on the head surface"; return;
    end
    direction = (center - closest) / norm(center - closest);
    [q, why, det] = solveStep(qPrev, tip + gp.stepM * direction, R, ctx, gp);
    if why ~= ""
        reason = why; detail = sprintf("retreat step %d: %s", step, det); return;
    end
    Q(:, end + 1) = q; %#ok<AGROW>
    qPrev = q;
end
reason = "IK"; detail = sprintf("clearance %.0f mm not reached within %.0f mm", 1000 * gp.clearM, 1000 * gp.maxRetreatM);
end

function [Q, ok, why] = jointLine(qa, qb, ctx, gp)
%JOINTLINE Straight joint-space line qa -> qb (excluding qa), every sample through the gates.
n = max(1, ceil(max(abs(qb - qa)) / gp.jointStepRad));
Q = zeros(6, 0); ok = true; why = "";
prev = qa;
for k = 1:n
    q = qa + (qb - qa) * k / n;
    [okK, w] = gates(q, prev, ctx, gp);
    if ~okK
        ok = false; why = sprintf("%s (sample %d/%d)", w, k, n); Q = zeros(6, 0); return;
    end
    Q(:, end + 1) = q; %#ok<AGROW>
    prev = q;
end
end

function [Q, reason, detail, QPartial] = shellArc(qa, qb, ctx, gp)
%SHELLARC Head-centred shell path between the two clear configurations (excluding qa, ending in qb).
Q = zeros(6, 0); reason = "OK"; detail = ""; QPartial = zeros(6, 0);
Ta = getTransform(ctx.robot, qa, char(ctx.endEffector));
Tb = getTransform(ctx.robot, qb, char(ctx.endEffector));
Ra = Ta(1:3, 1:3); Rb = Tb(1:3, 1:3);
pa = hairsim.getMountedClipperPoint(Ta, ctx.toolMount.T_ee_tool);
pb = hairsim.getMountedClipperPoint(Tb, ctx.toolMount.T_ee_tool);
cH = ctx.headCenterWorldM(:)';
rho = max(norm(pa - cH), norm(pb - cH)) + gp.shellMarginM;
ua = (pa - cH) / norm(pa - cH);
ub = (pb - cH) / norm(pb - cH);
ang = acos(min(1, max(-1, dot(ua, ub))));
axis = cross(ua, ub);
if norm(axis) < 1e-9
    axis = cross([0, 0, 1], ua);
    if norm(axis) < 1e-9, axis = cross([1, 0, 0], ua); end
end
axis = axis / norm(axis);
rel = rotm2axang(Ra' * Rb);

pts = zeros(0, 3); rots = zeros(3, 3, 0);
% 1) radial out to the shell, orientation held
nOut = max(1, ceil(abs(rho - norm(pa - cH)) / gp.stepM));
for k = 1:nOut
    pts(end + 1, :) = cH + ua * (norm(pa - cH) + (rho - norm(pa - cH)) * k / nOut); rots(:, :, end + 1) = Ra; %#ok<AGROW>
end
% 2) spherical arc on the shell, orientation slerped
nArc = max(1, ceil(rho * ang / gp.stepM));
for k = 1:nArc
    t = ang * k / nArc;
    u = ua * cos(t) + cross(axis, ua) * sin(t);
    pts(end + 1, :) = cH + rho * u; %#ok<AGROW>
    rots(:, :, end + 1) = Ra * axang2rotm([rel(1:3), rel(4) * k / nArc]); %#ok<AGROW>
end
% 3) radial back in to qb_clear, orientation Rb
nIn = max(1, ceil(abs(rho - norm(pb - cH)) / gp.stepM));
for k = 1:nIn
    pts(end + 1, :) = cH + ub * (rho + (norm(pb - cH) - rho) * k / nIn); rots(:, :, end + 1) = Rb; %#ok<AGROW>
end

qPrev = qa;
for k = 1:size(pts, 1)
    [q, why, det] = solveStep(qPrev, pts(k, :), rots(:, :, k), ctx, gp);
    if why ~= ""
        reason = why; detail = sprintf("shell sample %d/%d: %s", k, size(pts, 1), det); QPartial = Q; Q = zeros(6, 0); return;
    end
    Q(:, end + 1) = q; %#ok<AGROW>
    qPrev = q;
end
% blend from wherever the IK ended onto qb_clear (same gates)
[Qb, ok, why] = jointLine(qPrev, qb, ctx, gp);
if ~ok
    reason = "COLLISION"; detail = "blend onto qb_clear: " + why; Q = zeros(6, 0); return;
end
Q = [Q, Qb];
end
