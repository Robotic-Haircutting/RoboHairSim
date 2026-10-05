function adapter = createRobotAdapter(name, headScene, toolMount, options)
%CREATEROBOTADAPTER Build a ready-to-use robot/IK bootstrap for one named robot.
%   adapter = hairsim.robot.createRobotAdapter(name, headScene, toolMount)
%
%   This replaces the robot/IK bootstrap that used to be duplicated,
%   near-verbatim, in ~20+ files across matlab_ur5e (loadrobot + attach
%   clipper collision + inverseKinematics + the Home clearance fix-up +
%   the ceiling-mount Home). Adding a new robot means writing one
%   +hairsim/+robot/+adapters/<name>.m file (see ur5e.m for the reference
%   contract) and registering it -- never editing this function.
%
%   headScene is hairsim.createDefaultHeadScene's result (needs
%   headMeshWorld, headMeshSafetyWorld, headCenterWorldM). toolMount is
%   hairsim.motion.createClipperToolMount's result.
%
%   adapter fields: name, robot (rigidBodyTree, clipper collision
%   attached), plainRobot (fresh rigidBodyTree, no clipper collision --
%   for base-placement variants that attach it themselves after moving the
%   base), ik, endEffector, weights, numJoints, redundantJointIndices
%   (1:numJoints), qHome (safe at the default head scene; re-solved via IK
%   if the adapter's own tuned constant is missing, wrong-sized, or leaves
%   the clipper <1mm from the head), qCurrent (= qHome), qHomeCeiling (the
%   ceiling-mount study's Home; same fallback rule), toolMount,
%   headCollision (collision objects for headScene.headMeshWorld, computed
%   once here so callers building a planning context do not recompute it).
%
%   options.CollisionMarginM (default 0), options.MaxToolTipErrorM
%   (default 0.03): passed through to the Home clearance fix-up solve, same
%   defaults the original bootstrap used.

arguments
    name (1, 1) string
    headScene (1, 1) struct
    toolMount (1, 1) struct
    options.CollisionMarginM (1, 1) double {mustBeNonnegative} = 0
    options.MaxToolTipErrorM (1, 1) double {mustBeNonnegative} = 0.03
    options.HeadCollision = []
end

builderFn = hairsim.robot.getRobotAdapterBuilder(name);
raw = builderFn();

robot = loadrobot(raw.loadRobotName, 'DataFormat', 'column');
plainRobot = loadrobot(raw.loadRobotName, 'DataFormat', 'column');
robot = hairsim.motion.attachClipperCollisionToRobot(robot, raw.endEffector, toolMount);

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
% IK fix-up for it, as matlab_gen3's createGuiHomeConfiguration did.
homeIsTuned = numel(raw.qHomeConstant) == numJoints;
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;

center = hairsim.getMountedClipperPoint( ...
    getTransform(robot, qHome, char(raw.endEffector)), toolMount.T_ee_tool_center);
clearance = hairsim.evaluateClipperSphereMeshClearance( ...
    center, headScene.headMeshSafetyWorld, toolMount.toolClearanceRadiusM);
if ~homeIsTuned || clearance.surfaceClearanceM < 0.001
    safeTip = hairsim.path.createInitialClipperToolTip( ...
        headScene.headCenterWorldM, headScene.headMeshSafetyWorld, toolMount);
    [qSafe, ~, solved] = hairsim.motion.solveRobotForClipperTipWithOrientationFallback( ...
        robot, ik, raw.endEffector, [0.01, 0.01, 0.01, 5, 5, 5], qHome, safeTip, ...
        headScene.headCenterWorldM, toolMount.T_ee_tool, options.MaxToolTipErrorM, ...
        headCollision, toolMount, options.CollisionMarginM, ...
        'RedundantJointIndices', redundantJointIndices, 'InitialSeedConfiguration', qHome);
    if solved
        qHome = qSafe;
    else
        warning("hairsim:UnsafeInitialConfigurationUnresolved", ...
            "qHome's actual clearance is %.4f m for robot adapter '%s' and no safe " + ...
            "replacement was found via IK; strokes may be rejected as INVALID_START.", ...
            clearance.surfaceClearanceM, raw.name);
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
