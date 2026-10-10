function appState = createRobotAppState(adapterName, scene, options)
%CREATEROBOTAPPSTATE Build the Robot/Base subsystem's state: a
%   session-constant `modelContext` (RobotModelContext) plus a live `state`
%   (RobotState), starting at the "fixed" base placement.
%
%   appState fields:
%     modelContext  RobotModelContext: robot, endEffector, toolMount,
%                   headCollision, qHome, adapterName, rootDir,
%                   toolDisplayMesh (display-only; never passed to planning). Safe to use
%                   as part of a planning cache key -- it only changes when
%                   hairsim.app.robot.switchBase is called.
%     state         RobotState: qCommanded, qCurrent, basePlacementId,
%                   railThetaRad (nan unless basePlacementId starts with
%                   "rail"). Updated every tick by
%                   hairsim.app.robot.applyPoseCommand; also replaced
%                   wholesale by hairsim.app.robot.switchBase.
%
%   Everything else this struct carries (adapter, headScene, the
%   ceiling/rail option defaults) is PRIVATE bookkeeping switchBase needs
%   to rebuild modelContext/state later -- no other subsystem may read it;
%   they only ever receive appState.modelContext / appState.state.
%
%   options.CeilingHeightM (default 0.89, matching the overhead study) and
%   options.RailRadiusM/RailBaseOffsetM (defaults match
%   hairsim.robot.createRailContextBuilder) are remembered here so
%   hairsim.app.robot.switchBase doesn't need them repeated on every call.
%
%   options.ToolDisplay: "clipperHead" (default) draws the blade-head wedge
%   of scene.paths.clipperDisplayStl (hairsim.motion.createClipperDisplayMesh);
%   "model" draws the parametric clipper model itself. Display only --
%   cutting, clearance and collision always use the clipper model
%   (toolMount.tool). Falls back to "model" (with a warning) if the STL is
%   missing.

arguments
    adapterName (1, 1) string
    scene (1, 1) struct
    options.ToolMount = []
    options.ToolDisplay (1, 1) string {mustBeMember(options.ToolDisplay, ["clipperHead", "model"])} = "clipperHead"
    options.CollisionMarginM (1, 1) double {mustBeNonnegative} = 0
    options.MaxToolTipErrorM (1, 1) double {mustBeNonnegative} = 0.03
    options.CeilingHeightM (1, 1) double {mustBePositive} = 0.89
    options.RailRadiusM (1, 1) double {mustBePositive} = 0.60
    options.RailBaseOffsetM (1, 3) double {mustBeFinite} = [0, 0, -0.04]
end

toolMount = options.ToolMount;
if isempty(toolMount)
    toolMount = hairsim.motion.createClipperToolMount();
end
adapter = hairsim.robot.createRobotAdapter(adapterName, scene.headScene, toolMount, ...
    'CollisionMarginM', options.CollisionMarginM, 'MaxToolTipErrorM', options.MaxToolTipErrorM);

appState = struct();
appState.adapter = adapter;
appState.headScene = scene.headScene;
appState.rootDir = scene.rootDir;
appState.ceilingHeightM = options.CeilingHeightM;
appState.railRadiusM = options.RailRadiusM;
appState.railBaseOffsetM = options.RailBaseOffsetM;

appState.modelContext = struct( ...
    'robot', adapter.robot, ...
    'endEffector', adapter.endEffector, ...
    'toolMount', adapter.toolMount, ...
    'headCollision', {adapter.headCollision}, ...
    'qHome', adapter.qHome, ...
    'adapterName', adapter.name, ...
    'rootDir', scene.rootDir, ...
    'toolDisplayMesh', {createToolDisplayMesh(options.ToolDisplay, scene, adapter.toolMount)});
appState.state = struct( ...
    'qCommanded', adapter.qHome, ...
    'qCurrent', adapter.qHome, ...
    'basePlacementId', "fixed", ...
    'railThetaRad', nan);
end

function displayMesh = createToolDisplayMesh(toolDisplay, scene, toolMount)
displayMesh = [];
if toolDisplay ~= "clipperHead"
    return;
end
stlPath = "";
if isfield(scene, 'paths') && isfield(scene.paths, 'clipperDisplayStl')
    stlPath = string(scene.paths.clipperDisplayStl);
end
if strlength(stlPath) == 0 || ~isfile(stlPath)
    warning("hairsim:ClipperDisplayStlMissing", ...
        "Clipper display STL not found (%s); drawing the parametric clipper model instead.", stlPath);
    return;
end
displayMesh = hairsim.motion.createClipperDisplayMesh(stlPath, toolMount);
end
