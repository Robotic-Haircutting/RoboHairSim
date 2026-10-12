function teleopState = createTeleopState(scene, options)
%CREATETELEOPSTATE State of clipper Teleoperation: the clipper is driven
%   ON the scalp, not through free space. The user steers a surface state
%     surface.footWorldM  scalp point under the blade centre
%     surface.feedWorld   feed direction (TCP x), tangent to the scalp
%     surface.standoffM   blade height above the foot along the normal
%                         (tool.nominalStandoffM = touching, up to MaxStandoffM
%                         = hovering, which is how the clipper moves in air)
%     surface.rollRad     edge-cut roll about the feed axis
%   and every tick turns it into a TCP pose with the SAME pose builder,
%   contact resolution and safety gates the camClipper planner uses
%   (hairsim.tool.buildTcpPose, resolveContactStandoff,
%   hairsim.post.solveClipperPose). Nothing here moves the robot:
%   hairsim.app.teleop.stepTeleop proposes a PoseCommand, the entry applies
%   it like an Auto tick.
%
%   Call hairsim.app.teleop.syncToRobot before the first step and whenever
%   something else moved the robot, and hairsim.app.teleop.setHair whenever
%   the hair was (re)built.

arguments
    scene (1, 1) struct
    options.PlannerName (1, 1) string = "camClipperAgainstFlow"   % its tool = the cut rule
    options.MoveStepM (1, 1) double {mustBePositive} = 0.005
    options.YawStepRad (1, 1) double {mustBePositive} = deg2rad(15)
    options.StandoffStepM (1, 1) double {mustBePositive} = 0.010
    options.MaxStandoffM (1, 1) double {mustBePositive} = 0.080
    options.RollStepRad (1, 1) double {mustBePositive} = deg2rad(15)
    options.MaxRollRad (1, 1) double {mustBeNonnegative} = deg2rad(30)
    options.MaxTranslationPerTickM (1, 1) double {mustBePositive} = 0.010
    options.MaxRotationPerTickRad (1, 1) double {mustBePositive} = deg2rad(10)
end

teleopState = struct();
teleopState.tool = hairsim.planning.camClipperTool(options.PlannerName);
teleopState.headQuery = hairsim.tool.createHeadDistanceQuery(scene.headScene.headMeshWorld);
teleopState.headCenterWorldM = scene.headScene.headCenterWorldM;
teleopState.moveStepM = options.MoveStepM;
teleopState.yawStepRad = options.YawStepRad;
teleopState.standoffStepM = options.StandoffStepM;
teleopState.maxStandoffM = options.MaxStandoffM;
teleopState.rollStepRad = options.RollStepRad;
teleopState.maxRollRad = options.MaxRollRad;
teleopState.maxTranslationPerTickM = options.MaxTranslationPerTickM;
teleopState.maxRotationPerTickRad = options.MaxRotationPerTickRad;
% IK / safety parameters of the camClipper post-processor
teleopState.params = struct('ikWeights', [0.01, 0.01, 0.01, 5, 5, 5], 'maxToolTipErrorM', 0.002, ...
    'collisionMarginM', 0.001, 'maxJointStepRad', deg2rad(60));

teleopState.surface = struct('footWorldM', nan(1, 3), 'feedWorld', nan(1, 3), ...
    'standoffM', NaN, 'rollRad', 0);
teleopState.pending = hairsim.app.teleop.emptyInput();
teleopState.spaceHeld = false;
teleopState.settling = false;     % the robot is still turning/moving toward the surface state
teleopState.hairRoots = zeros(0, 3);
teleopState.hairFlow = zeros(0, 3);
teleopState.strandRegions = strings(0, 1);
teleopState.regionReachM = 0;
teleopState.regionName = "";       % "" = whole head (hairsim.app.teleop.setRegion)
teleopState.station = struct('robot', [], 'sctx', []);
teleopState.hud = struct('status', "-", 'standoffMm', NaN, 'inContact', false, ...
    'feedAngleDeg', NaN, 'clearanceMm', NaN, 'rollDeg', 0, 'inRegion', true, 'regionDistanceMm', 0);
end
