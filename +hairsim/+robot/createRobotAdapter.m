function adapter = createRobotAdapter(name, headScene, toolMount, options)
%CREATEROBOTADAPTER Build a ready-to-use robot/IK bootstrap for one named robot.
%   adapter = hairsim.robot.createRobotAdapter(name, headScene, toolMount)
%
%   One place for the robot bootstrap: loadrobot, the clipper collision
%   body, the IK solver, the Home clearance fix-up and the ceiling-mount
%   Home. Adding a new robot means writing one
%   +hairsim/+robot/+adapters/<name>.m file (see ur5e.m for the reference
%   contract) and registering it -- never editing this function.
%
%   headScene is hairsim.createDefaultHeadScene's result (needs
%   headMeshWorld, headCenterWorldM). toolMount is
%   hairsim.motion.createClipperToolMount's result.
%
%   adapter fields: name, robot (rigidBodyTree, clipper collision
%   attached), plainRobot (fresh rigidBodyTree, no clipper collision --
%   for base-placement variants that attach it themselves after moving the
%   base), ik, endEffector, weights, numJoints, redundantJointIndices
%   (1:numJoints), qHome (safe at the default head scene; re-solved via IK
%   if the adapter's own tuned constant is missing, wrong-sized, or leaves
%   the clipper model closer than options.HomeClearanceM to the head),
%   qCurrent (= qHome), qHomeCeiling (the adapter's ceiling-mount Home as
%   given, or the robot's homeConfiguration if missing or wrong-sized; no
%   clearance fix-up), toolMount,
%   headCollision (collision objects for headScene.headMeshWorld, computed
%   once here so callers building a planning context do not recompute it).
%
%   options.CollisionMarginM (default 0), options.MaxToolTipErrorM
%   (default 0.03), options.StartTipWorldM: the Home fix-up solve.

arguments
    name (1, 1) string
    headScene (1, 1) struct
    toolMount (1, 1) struct
    options.CollisionMarginM (1, 1) double {mustBeNonnegative} = 0
    options.MaxToolTipErrorM (1, 1) double {mustBeNonnegative} = 0.03
    options.HeadCollision = []
    options.HomeClearanceM (1, 1) double {mustBeNonnegative} = 0.001
    options.StartTipWorldM (1, 3) double = [0.4850, -0.0200, 0.2520]
end

builderFn = hairsim.robot.getRobotAdapterBuilder(name);
raw = builderFn();

robot = loadrobot(raw.loadRobotName, 'DataFormat', 'column');
plainRobot = loadrobot(raw.loadRobotName, 'DataFormat', 'column');

numJoints = numel(homeConfiguration(robot));
redundantJointIndices = 1:numJoints;

if isempty(options.HeadCollision)
    headCollision = hairsim.motion.createHeadCollisionModel(headScene.headMeshWorld).objects;
else
    headCollision = options.HeadCollision;
end
if ~iscell(headCollision)
    % Normalize to a cell regardless of caller input: struct()'s own
    % cell-broadcast rule would otherwise unwrap a 1x1 cell when building
    % `adapter` below, silently handing callers a bare collision object
    % instead of the cell every downstream collision check and
    % hairsim.robot.createRailContextBuilder's own concatenation expects.
    headCollision = {headCollision};
end

qHome = resolveHomeConstant(raw.qHomeConstant, numJoints, robot);
% An untuned robot (no/wrong-size qHomeConstant) falls back to its zero
% posture, which is nowhere near the head (e.g. kinovaGen3 points straight
% up), so the clearance test below would never trigger -- always run the
% IK fix-up for it.
homeIsTuned = numel(raw.qHomeConstant) == numJoints;
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;

% Home must leave the clipper clear of the head (clipper model vs the exact
% head mesh) and the links clear of the head hull; an untuned robot always
% gets a Home solved by IK at a start point pushed clear of the scalp.
tool = toolMount.tool;
headQuery = hairsim.tool.createHeadDistanceQuery(headScene.headMeshWorld);
clearance = homeClipperClearance(robot, qHome, raw.endEffector, tool, headQuery);
if ~homeIsTuned || clearance < options.HomeClearanceM
    [qSafe, solved] = solveSafeHome(robot, ik, raw, qHome, headScene, tool, headQuery, ...
        headCollision, options);
    if solved
        qHome = qSafe;
    else
        warning("hairsim:UnsafeInitialConfigurationUnresolved", ...
            "qHome's clipper clearance is %.4f m for robot adapter '%s' and no safe " + ...
            "replacement was found via IK.", clearance, raw.name);
    end
end
% Rebuild the IK solver fresh after the fix-up solve above: reusing the
% SAME inverseKinematics object for later planning solves measurably
% degrades them (see hairsim.motion.solveRobotForClipperTip's own history
% for the diagnosis this was ported from).
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;

qHomeCeiling = resolveHomeConstant(raw.qHomeCeiling, numJoints, plainRobot);

adapter = struct( ...
    'name', raw.name, ...
    'robot', robot, ...
    'plainRobot', plainRobot, ...
    'ik', ik, ...
    'endEffector', raw.endEffector, ...
    'weights', raw.weights, ...
    'numJoints', numJoints, ...
    'redundantJointIndices', redundantJointIndices, ...
    'qHome', qHome, ...
    'qCurrent', qHome, ...
    'qHomeCeiling', qHomeCeiling, ...
    'toolMount', toolMount, ...
    'headCollision', {headCollision});
end

function qHome = resolveHomeConstant(qHomeConstant, numJoints, robotForFallback)
if numel(qHomeConstant) == numJoints
    qHome = qHomeConstant(:);
else
    qHome = homeConfiguration(robotForFallback);
end
end

function c = homeClipperClearance(robot, q, endEffector, tool, headQuery)
T = getTransform(robot, q, char(endEffector)) * tool.T_ee_tcp;
r = hairsim.tool.evaluateClipperClearance(T, tool, headQuery, 'CapM', 0.2);
c = r.minM;
end

function [qSafe, solved] = solveSafeHome(robot, ik, raw, qHome, headScene, tool, headQuery, ...
        headCollision, options)
% IK for the TCP at a fixed start point near the head (facing the head
% centre, all roll candidates), moved out along the scalp normal in 5 mm
% steps until a candidate keeps the clipper options.HomeClearanceM clear
% and the links clear of the head hull.
qSafe = qHome;
solved = false;
tip = options.StartTipWorldM;
for step = 0:20
    [~, ~, ~, ~, candidates] = hairsim.motion.solveRobotForClipperTip( ...
        robot, ik, raw.endEffector, [0.01, 0.01, 0.01, 5, 5, 5], qHome, tip, ...
        headScene.headCenterWorldM, tool.T_ee_tcp, options.MaxToolTipErrorM, ...
        'HeadCollisionForIk', headCollision, 'CollisionMarginMForIk', options.CollisionMarginM, ...
        'InitialSeedConfiguration', qHome, 'RedundantJointIndices', 1:numel(qHome));
    for k = 1:numel(candidates)
        if candidates(k).solved && candidates(k).collisionClear && ...
                homeClipperClearance(robot, candidates(k).q, raw.endEffector, tool, headQuery) >= options.HomeClearanceM
            qSafe = candidates(k).q;
            solved = true;
            return;
        end
    end
    [dist, closest] = hairsim.tool.signedDistanceToHead(tip, headQuery);
    if ~(abs(dist) > 1e-9) || any(isnan(closest))
        return;
    end
    tip = tip + 0.005 * sign(dist) * (tip - closest) / norm(tip - closest);
end
end
