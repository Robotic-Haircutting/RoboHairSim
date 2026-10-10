function result = replayFullDemoBundle(bundleOrPath, options)
%REPLAYFULLDEMOBUNDLE Replay a bundle saved by hairsim.app.runFullDemoSession
%   (options.SaveBundlePath, or result.bundle passed directly) with ZERO
%   planning calls: no hairsim.app.planning.planHaircut, no
%   hairsim.planning.planSafeConfigurationTransition, not even a cache
%   lookup -- every joint configuration for every tick of the whole demo
%   (region strokes, inter-region bridges, rail fold/slide/unfold) was
%   already computed and stored by the run that built the bundle; this
%   function only ever calls hairsim.app.applyPoseTick (apply pose, step
%   hair, record, render) and, where the stored sample needs a different
%   base placement, the geometry-only rebuilds
%   hairsim.app.robot.applyRailStationForReplay / hairsim.app.robot.switchBase
%   -- neither of those solves IK or plans anything.
%
%   bundleOrPath: either the bundle struct itself (e.g. from a prior
%   result.bundle) or a string/char path to a .mat file saved with
%   SaveBundlePath (containing a variable named 'bundle').
%
%   options.DataDir/PlanningDataDir default to the bundle's own
%   bundle.meta values (recorded by the run that built it) -- pass them
%   explicitly only to point at a different data location.
%
%   result fields: recordingResult, numTicks, numCutTicks (same shape as
%   hairsim.app.runFullDemoSession's result, minus regionResults/bundle,
%   since a pure replay has no per-region planning outcomes of its own),
%   hairAppState, scene -- the final hair state and scene, ready to feed
%   straight into hairsim.app.renderMultiAngleSnapshot for a result image.

arguments
    bundleOrPath
    options.DataDir (1, 1) string = ""
    options.PlanningDataDir (1, 1) string = ""
    options.RunId (1, 1) string = ""
    options.ShowScene (1, 1) logical = true
    % Guard-comb height (m) of the geometric flat-blade cut model
    % (hairsim.hair.cutting.createGuardBladeCutModel). NaN (default) = the
    % bundle planner's comb-lift clipper cut model.
    options.GuardBladeHeightM (1, 1) double = nan
    % Optional video of the replay (needs ShowScene): every VideoFrameStride-th
    % tick is written to VideoPath (MPEG-4) at VideoFps. "" = no video.
    options.VideoPath (1, 1) string = ""
    options.VideoFrameStride (1, 1) double {mustBePositive, mustBeInteger} = 2
    options.VideoFps (1, 1) double {mustBePositive} = 30
    % Camera (needs ShowScene). "fixed" = the original single view; "follow"
    % = hairsim.app.view.computeFollowCameraPath: the camera turns to the
    % part of the head being cut. CameraOptions (struct) passes through to
    % that function (e.g. AzimuthOffsetsDeg, ElevationAddsDeg, DistanceM, ViewAngleDeg).
    options.CameraMode (1, 1) string {mustBeMember(options.CameraMode, ["fixed", "follow"])} = "fixed"
    options.CameraOptions (1, 1) struct = struct()
    % On-screen caption: region being cut, strands at target, length, rail angle.
    options.Captions (1, 1) logical = false
    % Seconds of camera orbit around the finished head at the end of the
    % video (no physics). 0 = none.
    options.EndOrbitS (1, 1) double {mustBeNonnegative} = 0
    % Camera check without the replay: render the scene (head + robot +
    % clipper, hair NOT simulated) at these bundle samples with the follow
    % camera into one tiled image at PreviewPath, then return.
    options.PreviewSamples (1, :) double = []
    options.PreviewPath (1, 1) string = ""
end

if isstring(bundleOrPath) || ischar(bundleOrPath)
    loaded = load(bundleOrPath, 'bundle');
    bundle = loaded.bundle;
else
    bundle = bundleOrPath;
end

dataDir = options.DataDir;
if strlength(dataDir) == 0
    dataDir = bundle.meta.dataDir;
end
planningDataDir = options.PlanningDataDir;
if strlength(planningDataDir) == 0
    planningDataDir = bundle.meta.planningDataDir;
end
runId = options.RunId;
if strlength(runId) == 0
    runId = bundle.meta.runId + "_replay";
end

sceneArgs = {};
if strlength(dataDir) > 0
    sceneArgs = [sceneArgs, {'DataDir', dataDir}];
end
if strlength(planningDataDir) > 0
    sceneArgs = [sceneArgs, {'PlanningDataDir', planningDataDir}];
end
scene = hairsim.api.loadScene(sceneArgs{:});

robotAppState = hairsim.app.robot.createRobotAppState(bundle.meta.adapterName, scene);
hairAppState = hairsim.app.hair.createHairAppState(scene);
if ~isnan(options.GuardBladeHeightM)
    hairAppState.hairLayer.cutting.model = hairsim.hair.cutting.createGuardBladeCutModel( ...
        scene.headScene.headMeshWorld, robotAppState.modelContext.toolMount, options.GuardBladeHeightM);
end
if isfield(bundle.meta, 'plannerName')
    % a camClipper-family bundle was cut by its preset's comb-lift clipper model
    cutModel = hairsim.hair.cutting.createCombLiftCutModelForPlanner(bundle.meta.plannerName, ...
        hairAppState.hairLayer.haircutState, scene.headScene.headMeshWorld);
    if ~isempty(cutModel)
        hairAppState.hairLayer.cutting.model = cutModel;
    end
end
initialActiveNodeCounts = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;

cameraPath = [];
if options.CameraMode == "follow" || ~isempty(options.PreviewSamples)
    T_ee_tcp = robotAppState.modelContext.toolMount.T_ee_tool;
    if isfield(bundle.meta, 'plannerName')   % older bundles do not record it
        [camTool, isCam] = hairsim.planning.camClipperTool(bundle.meta.plannerName);
        if isCam
            T_ee_tcp = camTool.T_ee_tcp;
        end
    end
    cameraArgs = namedargs2cell(options.CameraOptions);
    cameraPath = hairsim.app.view.computeFollowCameraPath(bundle, robotAppState, scene, T_ee_tcp, cameraArgs{:});
end
if ~isempty(options.PreviewSamples)
    result = previewCamera(bundle, robotAppState, hairAppState, scene, cameraPath, ...
        options.PreviewSamples, options.PreviewPath);
    return;
end
regionNames = containers.Map( ...
    {'scalp_back', 'scalp_left', 'scalp_right', 'scalp_top_left', 'scalp_top_right'}, ...
    {'Back', 'Left side', 'Right side', 'Top left', 'Top right'});

viewState = [];
captionHandle = [];
if options.ShowScene
    fig = figure('Name', 'HairSimApp: full demo replay (bundle, no planning)', 'Color', 'w');
    ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.06, 0.08, 0.88, 0.86]);
    hairsim.hideAxesToolbar(ax);
    axis(ax, 'equal');
    view(ax, 135, 15);
    viewState = hairsim.app.view.createViewState(ax);
    if options.Captions
        captionHandle = annotation(fig, 'textbox', [0.01, 0.86, 0.6, 0.13], 'String', '', ...
            'FontSize', 13, 'FontName', 'Helvetica', 'EdgeColor', 'none', ...
            'BackgroundColor', [1, 1, 1], 'FaceAlpha', 0.75, 'Interpreter', 'none');
    end
end
needsCut = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts > ...
    bundle.lengthGoal{1}.targetActiveNodeCounts;
video = [];
if strlength(options.VideoPath) > 0 && options.ShowScene
    video = VideoWriter(char(options.VideoPath), 'MPEG-4');
    video.FrameRate = options.VideoFps;
    open(video);
end

recordingState = hairsim.app.recording.createRecordingState( ...
    scene.rootDir, numel(robotAppState.state.qCurrent), ...
    robotAppState.modelContext.toolMount, 'RunId', runId);
recordingState = hairsim.app.recording.startSession(recordingState, ...
    numel(robotAppState.state.qCurrent), robotAppState.modelContext.toolMount.T_ee_tool);

headRadiusApproxM = 0.5 * max(scene.headScene.headMeshWorld.bboxSize);
floorZ = scene.headScene.headMeshWorld.bboxMin(3) - 0.35 * headRadiusApproxM;

numTicks = 0;
numCutTicks = 0;
numSamples = size(bundle.qCandidate, 2);
appliedRailThetaRad = nan;
for sampleIdx = 1:numSamples
    % Only rebuild the robot model when the station/angle actually
    % changed from the previous tick -- hairsim.app.robot.
    % applyRailStationForReplay/switchBase each hand back a NEW
    % rigidBodyTree object, and hairsim.motion.showClipperRobot's cheap
    % FastUpdate redraw path only fires when the SAME object is reused
    % tick to tick. Rebuilding unconditionally on every rail-mode sample
    % (even when the angle hadn't moved) forced a full mesh redraw every
    % single tick of the whole rail replay -- the real cause of the slow,
    % stuttery playback, not a rendering leak (hairsim.motion.
    % showClipperRobot already deletes the previous robot's handles before
    % every redraw, so nothing was actually left behind uncleaned).
    railThetaRad = bundle.railThetaRad(sampleIdx);
    if ~isnan(railThetaRad)
        if isnan(appliedRailThetaRad) || abs(railThetaRad - appliedRailThetaRad) > 1e-9
            robotAppState = hairsim.app.robot.applyRailStationForReplay(robotAppState, railThetaRad);
            appliedRailThetaRad = railThetaRad;
        end
    elseif bundle.basePlacementId(sampleIdx) ~= robotAppState.state.basePlacementId
        robotAppState = hairsim.app.robot.switchBase(robotAppState, bundle.basePlacementId(sampleIdx));
        appliedRailThetaRad = nan;
    end

    % lengthGoal controls how far applyClipperHaircutStep actually cuts
    % each strand (not just a reporting/summary field) -- it must be
    % re-applied exactly as it was during the original run, not left at
    % whatever hairsim.app.hair.createHairAppState's fresh default is.
    lengthGoal = bundle.lengthGoal{sampleIdx};
    hairAppState.hairLayer.haircutState.lengthGoal = lengthGoal;
    hairAppState.hairLayer.haircutState.params.targetHairLengthM = lengthGoal.targetLengthM;
    hairAppState.hairLayer.haircutState.params.targetHairLengthToleranceM = lengthGoal.toleranceM;

    poseCommand = struct( ...
        'qCandidate', bundle.qCandidate(:, sampleIdx), 'solved', true, 'toolTipWorldM', zeros(1, 3), ...
        'statusLabel', bundle.statusLabel(sampleIdx), 'toolTipErrorM', 0, 'targetTipWorldM', zeros(1, 3), ...
        'desiredTipWorldM', zeros(1, 3), 'safeTipWorldM', zeros(1, 3), 'governorActive', false, ...
        'cutFlag', bundle.cutFlag(sampleIdx));
    [robotAppState, hairAppState, recordingState, tickCounted] = hairsim.app.applyPoseTick( ...
        poseCommand, robotAppState, hairAppState, recordingState, viewState, scene, floorZ);
    numTicks = numTicks + tickCounted;
    if bundle.cutFlag(sampleIdx)
        numCutTicks = numCutTicks + 1;
    end
    if ~isempty(viewState) && ~isempty(cameraPath)
        applyCamera(ax, cameraPath.cameraPosition(sampleIdx, :), cameraPath.cameraTarget, cameraPath.viewAngleDeg);
    end
    if ~isempty(captionHandle)
        hs = hairAppState.hairLayer.haircutState;
        atTarget = nnz(needsCut & hs.hairState.activeNodeCounts <= hs.lengthGoal.targetActiveNodeCounts);
        captionHandle.String = captionText(cameraPath, sampleIdx, regionNames, atTarget, nnz(needsCut), ...
            lengthGoal.targetLengthM, railThetaRad, bundle.cutFlag(sampleIdx));
    end
    if ~isempty(video) && (mod(sampleIdx, options.VideoFrameStride) == 0 || sampleIdx == numSamples)
        writeVideo(video, fitFrame(captureFrame(fig), video));
    end
end
if ~isempty(video) && options.EndOrbitS > 0
    % orbit the finished head (no physics): one full turn from the last view
    numFrames = max(2, round(options.EndOrbitS * options.VideoFps));
    [az0, ~] = view(ax);
    target = scene.headScene.headCenterWorldM(:)';
    distance = 1.0;
    viewAngle = 24;
    if ~isempty(cameraPath)
        az0 = cameraPath.azimuthDeg(end);
        distance = norm(cameraPath.cameraPosition(end, :) - target);
        viewAngle = cameraPath.viewAngleDeg;
    end
    % the robot parks next to the head: hide it (and the clipper) so the
    % orbit shows only the finished head and hair
    robotHandles = gobjects(0);
    if isappdata(ax, 'hairsimRobotVisualHandles')
        robotHandles = getappdata(ax, 'hairsimRobotVisualHandles');
    end
    robotHandles = [robotHandles(:); findall(ax, 'Tag', 'HairSimClipperHead')];
    set(robotHandles(isgraphics(robotHandles)), 'Visible', 'off');
    if ~isempty(captionHandle)
        hs = hairAppState.hairLayer.haircutState;
        captionHandle.String = sprintf('Finished: %d / %d strands at target length %.0f mm', ...
            nnz(needsCut & hs.hairState.activeNodeCounts <= hs.lengthGoal.targetActiveNodeCounts), ...
            nnz(needsCut), 1000 * hs.lengthGoal.targetLengthM);
    end
    for f = 1:numFrames
        az = az0 + 360 * f / numFrames;
        dir = [cosd(25) * sind(az), -cosd(25) * cosd(az), sind(25)];
        applyCamera(ax, target + distance * dir, target, viewAngle);
        writeVideo(video, fitFrame(captureFrame(fig), video));
    end
end
if ~isempty(video)
    close(video);
end

recordingState = hairsim.app.recording.endSession(recordingState);
config = hairsim.control.createRobotHaircutConfig(scene.paths, recordingState.runPaths, ...
    'Mode', "auto-benchmark", 'ToolMount', robotAppState.modelContext.toolMount, ...
    'TargetHairLengthM', bundle.meta.targetLengthM, ...
    'TargetHairLengthToleranceM', hairAppState.hairLayer.haircutState.params.targetHairLengthToleranceM);
[recordingState, recordingResult] = hairsim.app.recording.finalizeSession(recordingState, ...
    robotAppState.modelContext, scene.paths, scene.headScene.headMeshWorld, ...
    initialActiveNodeCounts, hairAppState.hairLayer.haircutState, {}, config); %#ok<ASGLU>

% A pure replay has no per-region planning outcomes of its own (see the
% header), so only the global metrics are written here -- the same
% per-region breakdown from the original build run's demo_stats.json still
% applies unchanged, since replay reproduces it exactly.
statsDir = recordingState.runPaths.runDir;
fid = fopen(fullfile(statsDir, 'demo_stats.json'), 'w');
fprintf(fid, '%s', jsonencode(struct('runId', runId, 'global', recordingResult.metrics), 'PrettyPrint', true));
fclose(fid);

result = struct('recordingResult', recordingResult, 'numTicks', numTicks, 'numCutTicks', numCutTicks, ...
    'hairAppState', hairAppState, 'scene', scene, 'statsDir', statsDir);
end

function applyCamera(ax, position, target, viewAngleDeg)
campos(ax, position);
camtarget(ax, target);
camup(ax, [0, 0, 1]);
camva(ax, viewAngleDeg);
end

function text = captionText(cameraPath, sampleIdx, regionNames, atTarget, numNeeds, targetLengthM, railThetaRad, cutting)
regionLine = 'Moving';
if ~isempty(cameraPath) && cameraPath.region(sampleIdx) ~= ""
    key = char(cameraPath.region(sampleIdx));
    name = key;
    if isKey(regionNames, key)
        name = regionNames(key);
    end
    order = find(cameraPath.regionOrder == cameraPath.region(sampleIdx), 1);
    regionLine = sprintf('Region %d/%d: %s', order, numel(cameraPath.regionOrder), name);
    if ~cutting
        regionLine = [regionLine, '  (moving)'];
    end
end
railLine = '';
if isfinite(railThetaRad)
    railLine = sprintf('   |   rail %.0f deg', mod(rad2deg(railThetaRad), 360));
end
text = sprintf('%s\nCut to target: %d / %d strands   |   length %.0f mm%s', ...
    regionLine, atTarget, numNeeds, 1000 * targetLengthM, railLine);
end

function result = previewCamera(bundle, robotAppState, hairAppState, scene, cameraPath, samples, outPath)
% Scene at the given samples with the follow camera, each rendered in its
% own figure (a manual camera lets graphics spill over subplot borders)
% and tiled into one image; hair shown in its INITIAL state (not
% simulated) -- for checking camera angles only.
samples = samples(samples >= 1 & samples <= size(bundle.qCandidate, 2));
tiles = cell(1, numel(samples));
applied = nan;
for i = 1:numel(samples)
    k = samples(i);
    theta = bundle.railThetaRad(k);
    if ~isnan(theta) && (isnan(applied) || abs(theta - applied) > 1e-9)
        robotAppState = hairsim.app.robot.applyRailStationForReplay(robotAppState, theta);
        applied = theta;
    end
    ctx = robotAppState.modelContext;
    q = bundle.qCandidate(:, k);
    T_world_ee = getTransform(ctx.robot, q, char(ctx.endEffector));
    fig = figure('Color', 'w', 'Position', [50, 50, 640, 480], 'Visible', 'off');
    ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.02, 0.02, 0.96, 0.9]);
    viewState = hairsim.app.view.createViewState(ax);
    hairsim.app.view.renderScene(viewState, struct('robot', ctx.robot, 'qCurrent', q, ...
        'headMeshWorld', scene.headScene.headMeshWorld, 'T_world_ee_actual', T_world_ee, ...
        'toolMount', ctx.toolMount, 'toolDisplayMesh', {ctx.toolDisplayMesh}, ...
        'cutFlag', bundle.cutFlag(k), 'solved', true, 'toolTipErrorM', 0, 'poseStatus', "", 'sceneZMaxM', 0.65));
    hairsim.app.view.renderHairOverlay(viewState, hairAppState.hairLayer.haircutState);
    applyCamera(ax, cameraPath.cameraPosition(k, :), cameraPath.cameraTarget, cameraPath.viewAngleDeg);
    title(ax, sprintf('sample %d  %s  az %.0f el %.0f', k, cameraPath.region(k), ...
        mod(cameraPath.azimuthDeg(k), 360), cameraPath.elevationDeg(k)), 'Interpreter', 'none');
    tiles{i} = captureFrame(fig);
    close(fig);
end
numCols = ceil(sqrt(numel(tiles)));
numRows = ceil(numel(tiles) / numCols);
[h, w, ~] = size(tiles{1});
canvas = uint8(255 * ones(numRows * h, numCols * w, 3));
for i = 1:numel(tiles)
    r = floor((i - 1) / numCols);
    c = mod(i - 1, numCols);
    tile = imresize_(tiles{i}, h, w);
    canvas(r * h + (1:h), c * w + (1:w), :) = tile;
end
imwrite(canvas, outPath);
result = struct('cameraPath', cameraPath, 'previewPath', outPath);
end

function out = imresize_(img, h, w)
% crop/pad to h x w (frame sizes can differ by a pixel)
out = uint8(255 * ones(h, w, 3));
hh = min(h, size(img, 1));
ww = min(w, size(img, 2));
out(1:hh, 1:ww, :) = img(1:hh, 1:ww, :);
end

function img = captureFrame(fig)
% A video frame rendered offscreen from the current scene. getframe right
% after a scene update can grab a half-finished WebGL frame (in
% matlab -batch: unlit grey head, stray translucent bands and dotted lines
% that are not scene objects); print re-renders the whole figure.
img = print(fig, '-RGBImage', '-r0');
end

function img = fitFrame(img, video)
% The video's frame size is fixed by its first frame; a later print can come
% back a few pixels off (window or display scale change mid-replay): pad with
% white / crop to that size so writeVideo never rejects the frame.
if video.FrameCount == 0
    return
end
h = video.Height;
w = video.Width;
img = img(1:min(end, h), 1:min(end, w), :);
img(end + 1:h, :, :) = 255;
img(:, end + 1:w, :) = 255;
end
