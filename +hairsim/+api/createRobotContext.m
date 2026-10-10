function context = createRobotContext(adapterName, scene, options)
%CREATEROBOTCONTEXT Build the planner context struct from a named robot
%   adapter and a hairsim.api.loadScene result -- the library-level
%   equivalent of the robot/IK bootstrap every GUI script used to inline.
%
%   context = hairsim.api.createRobotContext("ur5e", scene) is the
%   'context' argument every registered region planner expects: robot, endEffector, toolMount,
%   headMeshWorld, headMeshSafetyWorld, headCollision, headCenterWorldM,
%   R_world_head, rootDir, qHome, qCurrent, redundantJointIndices,
%   numJoints.
%
%   options.ToolMount overrides hairsim.motion.createClipperToolMount's
%   default. Other options pass through to
%   hairsim.robot.createRobotAdapter (CollisionMarginM, MaxToolTipErrorM).
%
%   options.RailMode (default false): also attach context.railContextBuilder
%   (hairsim.robot.createRailContextBuilder's closure) and
%   context.railThetaCurrentRad (= options.RailThetaRad, the initial
%   station), which the post-processor's rail station search requires.
%   options.RailRadiusM/RailBaseOffsetM pass through to
%   createRailContextBuilder.

arguments
    adapterName (1, 1) string
    scene (1, 1) struct
    options.ToolMount = []
    options.CollisionMarginM (1, 1) double {mustBeNonnegative} = 0
    options.MaxToolTipErrorM (1, 1) double {mustBeNonnegative} = 0.03
    options.RailMode (1, 1) logical = false
    options.RailThetaRad (1, 1) double {mustBeFinite} = pi
    options.RailRadiusM (1, 1) double {mustBePositive} = 0.60
    options.RailBaseOffsetM (1, 3) double {mustBeFinite} = [0, 0, -0.04]
end

toolMount = options.ToolMount;
if isempty(toolMount)
    toolMount = hairsim.motion.createClipperToolMount();
end

adapter = hairsim.robot.createRobotAdapter(adapterName, scene.headScene, toolMount, ...
    'CollisionMarginM', options.CollisionMarginM, 'MaxToolTipErrorM', options.MaxToolTipErrorM);

context = struct();
context.robot = adapter.robot;
context.endEffector = adapter.endEffector;
context.toolMount = adapter.toolMount;
context.headMeshWorld = scene.headScene.headMeshWorld;
context.headMeshSafetyWorld = scene.headScene.headMeshSafetyWorld;
context.headCollision = adapter.headCollision;
context.headCenterWorldM = scene.headScene.headCenterWorldM;
context.R_world_head = scene.headScene.R_world_head;
context.rootDir = scene.rootDir;
context.qHome = adapter.qHome;
context.qCurrent = adapter.qCurrent;
context.redundantJointIndices = adapter.redundantJointIndices;
context.numJoints = adapter.numJoints;
context.adapterName = adapter.name;
context.plainRobot = adapter.plainRobot;
context.qHomeCeiling = adapter.qHomeCeiling;

if options.RailMode
    context.railContextBuilder = hairsim.robot.createRailContextBuilder( ...
        adapter, scene.headScene, 'RadiusM', options.RailRadiusM, ...
        'BaseOffsetM', options.RailBaseOffsetM);
    context.railThetaCurrentRad = options.RailThetaRad;
    built = context.railContextBuilder(options.RailThetaRad);
    context.robot = built.robot;
    context.headCollision = built.headCollision;
end
end
