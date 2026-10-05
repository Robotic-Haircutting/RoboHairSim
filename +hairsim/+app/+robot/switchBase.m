function appState = switchBase(appState, basePlacementId)
%SWITCHBASE Rebuild RobotModelContext (robot/headCollision/qHome) for a new
%   base placement and reset RobotState to that placement's own Home.
%   Ported from run_robot_haircut.m's applyBaseSetup.
%
%   basePlacementId: "fixed" | "ceiling" | "rail<thetaDeg>" (e.g. "rail135").
%
%   Per the session lifecycle invariant (see docs/EXTENDING.md /
%   the plan's "Session lifecycle" section): the App shell must not call
%   this while session.phase == "executing", and must treat the current
%   plan as invalidated after calling this (a plan is only valid for the
%   RobotModelContext it was made against).

arguments
    appState (1, 1) struct
    basePlacementId (1, 1) string
end

adapter = appState.adapter;
headScene = appState.headScene;
headOriginM = headScene.T_world_head(1:3, 4)';
railContextBuilder = [];

if basePlacementId == "fixed"
    robot = adapter.robot;
    headCollision = adapter.headCollision;
    qHome = adapter.qHome;
    railThetaRad = nan;
elseif basePlacementId == "ceiling"
    T_world_base = trvec2tform(headOriginM + [0, 0, appState.ceilingHeightM]) * axang2tform([0, 1, 0, pi]);
    robot = hairsim.motion.attachClipperCollisionToRobot( ...
        hairsim.motion.applyRobotBaseTransform(adapter.plainRobot, T_world_base), ...
        adapter.endEffector, adapter.toolMount);
    headCollision = adapter.headCollision;
    qHome = adapter.qHomeCeiling;
    railThetaRad = nan;
elseif startsWith(basePlacementId, "rail")
    railThetaRad = deg2rad(str2double(extractAfter(basePlacementId, "rail")));
    railContextBuilder = hairsim.robot.createRailContextBuilder(adapter, headScene, ...
        'RadiusM', appState.railRadiusM, 'BaseOffsetM', appState.railBaseOffsetM);
    built = railContextBuilder(railThetaRad);
    robot = built.robot;
    headCollision = built.headCollision;
    qHome = adapter.qHome;
else
    error("hairsim:UnknownBasePlacement", ...
        "basePlacementId must be ""fixed"", ""ceiling"", or ""rail<thetaDeg>"" (got ""%s"").", basePlacementId);
end

homeClear = hairsim.motion.validateClipperRobotCollision(robot, qHome, ...
    adapter.endEffector, adapter.toolMount, headCollision, 0, true);
if ~homeClear
    warning("hairsim:BaseHomeNotClear", ...
        "Home for base placement '%s' is not collision-free; plans will start from it anyway.", basePlacementId);
end

appState.modelContext.robot = robot;
appState.modelContext.headCollision = headCollision;
appState.modelContext.qHome = qHome;
% hairsim.planning.transitStrategies.railSlide (the "railslide" preset)
% requires these two fields on the context it receives; absent for
% "fixed"/"ceiling" so that preset's own isfield check fails loudly rather
% than silently reusing a stale rail builder from an earlier switch.
if isfield(appState.modelContext, 'railContextBuilder')
    appState.modelContext = rmfield(appState.modelContext, 'railContextBuilder');
end
if isfield(appState.modelContext, 'railThetaCurrentRad')
    appState.modelContext = rmfield(appState.modelContext, 'railThetaCurrentRad');
end
if ~isempty(railContextBuilder)
    appState.modelContext.railContextBuilder = railContextBuilder;
    appState.modelContext.railThetaCurrentRad = railThetaRad;
end
appState.state = struct( ...
    'qCommanded', qHome, ...
    'qCurrent', qHome, ...
    'basePlacementId', basePlacementId, ...
    'railThetaRad', railThetaRad);
end
