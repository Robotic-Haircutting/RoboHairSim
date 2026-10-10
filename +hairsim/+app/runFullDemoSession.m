function result = runFullDemoSession(options)
%RUNFULLDEMOSESSION Plan + run all five scalp regions back to back in one
%   continuous session: same scene/robot/hair/recording/view state carried
%   across regions (hair cut by region N is visible when region N+1 plans
%   and runs), one consolidated recording for the whole demo.
%
%   Between regions the robot moves with the same retreat -> transfer ->
%   approach connector the post-processor uses inside a region
%   (hairsim.planning.planSafeConfigurationTransition: clipper model vs the
%   exact head mesh, links vs the head hull + environment, arm + clipper
%   self collision on every sample). If a connection is infeasible for a
%   region pair, that one boundary falls back to a teleport and is flagged
%   in result.regionResults.bridgeWarning (the demo is not aborted).
%
%   options.BasePlacementId selects the base placement for the whole demo:
%     "fixed"       (default) -- the adapter's normal fixed mount.
%     "ceiling"     -- the overhead mount, same placement for all regions.
%     "rail<deg>"   -- one rail station (e.g. "rail180") for all regions.
%     "railPerRegion" -- each region uses ITS OWN frozen best rail station
%       from the 2026-09-25 matlab_ur5e rail-placement study (135 deg for
%       scalp_top_left/scalp_left, 150 for scalp_right, 180 for
%       scalp_back/scalp_top_right -- see regionRailStationDeg below).
%       When consecutive regions in options.RegionNames sit at different
%       stations, the robot folds back to qHome via
%       hairsim.planning.planSafeConfigurationTransition (the same
%       region/lane-agnostic retreat -> joint-line/shell-arc -> approach
%       connector the 2026-09-25 rail study itself used for Home
%       connections -- direct()'s LIFT/TRANSFER/LOWER assumes scalp-
%       following geometry on both ends, which Home doesn't have, and was
%       found NOT reliable for this leg), the rail is animated sliding
%       station to station in options.RailSlideStepDeg steps (q held at
%       qHome throughout -- qHome was verified collision-free across the
%       whole 120-180 deg sweep in that study; a per-step arm self-
%       collision + robot-vs-head check still runs here as a live sanity
%       check, not blind trust), then hairsim.app.robot.switchBase
%       finalizes the new station and the robot unfolds from qHome into
%       the next region's first stroke via the same
%       planSafeConfigurationTransition connector. Regions sharing a
%       station are still bridged arm-to-arm via direct() exactly like the
%       fixed-base case, no rail motion. For best results with
%       "railPerRegion", order
%       options.RegionNames so same-station regions are adjacent (e.g.
%       group by 135/150/180) -- an order that revisits a station after
%       leaving it will slide the rail back and forth.
%
%   A region whose plan comes back not executable is skipped (recorded in
%   result.regionResults with executable=false) rather than aborting the
%   whole demo; the bridge into the NEXT executable region then starts
%   from wherever the robot last actually was.
%
%   After the last region, the robot returns to Home (the same
%   planSafeConfigurationTransition connector used for every other Home
%   leg) and the demo keeps stepping hair/debris physics, arm held at
%   Home, until the debris list is empty (capped at 300 extra ticks) --
%   otherwise the demo would simply stop mid-stroke with the arm still
%   over the head and the last cut's hair clippings still airborne.
%
%   Every tick (region-execution replay sample, cross-region bridge
%   sample, rail-fold/slide/unfold sample) passes through the one
%   applyAndRecordTick choke point, which -- in addition to its existing
%   job -- appends {qCandidate, railThetaRad, basePlacementId, cutFlag,
%   statusLabel} to a flat, ordered bundle. This bundle is EVERYTHING
%   needed to replay the whole demo with zero planning/IK calls (see
%   hairsim.app.replayFullDemoBundle): it is returned as result.bundle and,
%   if options.SaveBundlePath is non-empty, also saved there as a .mat
%   (variable name 'bundle') together with the adapter/data-dir metadata
%   needed to rebuild the scene for a later replay.
%
%   result fields: regionResults (1xN struct: regionName, executable,
%   message, numChainedStrokes, bridgeWarning, removedNodeCount,
%   coverageScalpPct/SafePct/ExecPct/ChainPct -- the last four straight
%   from regionPlan.coverage, nan for a skipped region), recordingResult,
%   numTicks, numCutTicks, bundle, hairAppState, scene (ready to feed into
%   hairsim.app.renderMultiAngleSnapshot for a result image), statsDir (the
%   run's output folder, where demo_region_stats.csv and demo_stats.json
%   -- the same regionResults plus recordingResult.metrics -- are written
%   alongside the usual metrics.json/summary.txt).

arguments
    options.AdapterName (1, 1) string = "ur5e"
    options.RegionNames (1, :) string = ...
        ["scalp_back", "scalp_left", "scalp_top_left", "scalp_top_right", "scalp_right"]
    options.TargetLengthM (1, 1) double {mustBeNonnegative} = 0.005
    options.PlannerName (1, 1) string = "camClipperAgainstFlow"
    options.PlanTransits (1, 1) logical = true
    options.LaneIndices (1, :) double = []
    options.DataDir (1, 1) string = ""
    options.PlanningDataDir (1, 1) string = ""
    options.ConfigDir (1, 1) string = ""
    options.RunId (1, 1) string = ""
    options.ShowScene (1, 1) logical = true
    options.InitialQRad (1, :) double = []
    options.BasePlacementId (1, 1) string = "fixed"
    options.RailSlideStepDeg (1, 1) double {mustBePositive} = 2
    options.SaveBundlePath (1, 1) string = ""
end

sceneArgs = {};
if strlength(options.DataDir) > 0
    sceneArgs = [sceneArgs, {'DataDir', options.DataDir}];
end
if strlength(options.PlanningDataDir) > 0
    sceneArgs = [sceneArgs, {'PlanningDataDir', options.PlanningDataDir}];
end
scene = hairsim.api.loadScene(sceneArgs{:});

railPerRegion = options.BasePlacementId == "railPerRegion";

robotAppState = hairsim.app.robot.createRobotAppState(options.AdapterName, scene);
if railPerRegion
    initialBasePlacementId = "rail" + regionRailStationDeg(options.RegionNames(1));
else
    initialBasePlacementId = options.BasePlacementId;
end
if initialBasePlacementId ~= "fixed"
    robotAppState = hairsim.app.robot.switchBase(robotAppState, initialBasePlacementId);
end
if ~isempty(options.InitialQRad)
    robotAppState.state.qCurrent = options.InitialQRad(:);
    robotAppState.state.qCommanded = options.InitialQRad(:);
end
hairAppState = hairsim.app.hair.createHairAppState(scene);
% camClipper: the plan chooses its own rail stations per stroke and the hair
% is cut by the comb-lift clipper model (the same predicate the planner used).
[clipperTool, isCamClipper] = hairsim.planning.camClipperTool(options.PlannerName);
if ~isCamClipper
    error("hairsim:NotAClipperPlanner", "'%s' is not a camClipper-family planner.", options.PlannerName);
end
if isCamClipper
    hairAppState.hairLayer.cutting.model = hairsim.hair.cutting.createCombLiftCutModelForPlanner( ...
        options.PlannerName, hairAppState.hairLayer.haircutState, scene.headScene.headMeshWorld);
    if isfinite(robotAppState.state.railThetaRad)
        % the GUI Home leaves the real blade inside the head: every Home leg
        % of this demo (fold, rail slide, closing return) uses the
        % clipper-safe park pose instead (hairsim.post.findClipperParkConfiguration)
        parkCtx = hairsim.api.createRobotContext(options.AdapterName, scene, 'RailMode', true, ...
            'RailThetaRad', robotAppState.state.railThetaRad);
        qPark = hairsim.post.findClipperParkConfiguration(parkCtx, clipperTool, ...
            hairsim.tool.createHeadDistanceQuery(scene.headScene.headMeshWorld), [180, 165, 150, 135, 195, 210, 120, 225, 240]);
        robotAppState.homeOverride = qPark;
        robotAppState = applyHomeOverride(robotAppState);
    end
end
initialActiveNodeCounts = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;

configDir = options.ConfigDir;
if strlength(configDir) == 0
    configDir = fullfile(scene.rootDir, 'config');
end
planningState = hairsim.app.planning.createAutoPlanningState(configDir);

viewState = [];
if options.ShowScene
    fig = figure('Name', 'HairSimApp: full demo (all regions)', 'Color', 'w');
    ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.06, 0.08, 0.88, 0.86]);
    hairsim.hideAxesToolbar(ax);
    axis(ax, 'equal');
    view(ax, 135, 15);
    viewState = hairsim.app.view.createViewState(ax);
end

recordingState = hairsim.app.recording.createRecordingState( ...
    scene.rootDir, numel(robotAppState.state.qCurrent), ...
    robotAppState.modelContext.toolMount, 'RunId', options.RunId);
recordingState = hairsim.app.recording.startSession(recordingState, ...
    numel(robotAppState.state.qCurrent), robotAppState.modelContext.toolMount.T_ee_tool);

headRadiusApproxM = 0.5 * max(scene.headScene.headMeshWorld.bboxSize);
floorZ = scene.headScene.headMeshWorld.bboxMin(3) - 0.35 * headRadiusApproxM;

numTicks = 0;
numCutTicks = 0;
previousLastStroke = [];
previousBridgeLabel = "";
bridgeFromHome = false;
regionResults = struct('regionName', {}, 'executable', {}, 'message', {}, ...
    'numChainedStrokes', {}, 'bridgeWarning', {}, 'removedNodeCount', {}, ...
    'coverageScalpPct', {}, 'coverageSafePct', {}, 'coverageExecPct', {}, 'coverageChainPct', {});
bundle = struct('qCandidate', {{}}, 'railThetaRad', {{}}, 'basePlacementId', {{}}, ...
    'cutFlag', {{}}, 'statusLabel', {{}}, 'lengthGoal', {{}});

for regionName = options.RegionNames
    bridgeWarning = "";

    plannerNameForRegion = options.PlannerName;
    planRequest = struct( ...
        'robotModel', robotAppState.modelContext, ...
        'qCurrent', robotAppState.state.qCurrent, ...
        'headMeshWorld', scene.headScene.headMeshWorld, ...
        'headCenterWorldM', scene.headScene.headCenterWorldM, ...
        'rWorldHead', scene.headScene.R_world_head, ...
        'basePlacementId', robotAppState.state.basePlacementId, ...
        'haircutState', hairAppState.hairLayer.haircutState, ...
        'regionName', regionName, ...
        'targetLengthM', options.TargetLengthM, ...
        'plannerName', plannerNameForRegion, ...
        'options', struct('PlanTransits', options.PlanTransits, 'LaneIndices', options.LaneIndices, ...
            'OnProgress', @(msg) fprintf('[%s] %s\n', regionName, msg)), ...
        'useCache', true);
    fprintf('--- Planning %s (%s) ---\n', regionName, robotAppState.state.basePlacementId);
    [planningState, planResult] = hairsim.app.planning.planHaircut(planningState, planRequest);
    regionPlan = planResult.regionPlan;

    if ~regionPlan.executable
        warning("hairsim:FullDemoRegionNotExecutable", "%s: %s", regionName, regionPlan.message);
        regionResults(end + 1) = struct('regionName', regionName, 'executable', false, ...
            'message', regionPlan.message, 'numChainedStrokes', 0, 'bridgeWarning', bridgeWarning, ...
            'removedNodeCount', 0, 'coverageScalpPct', nan, 'coverageSafePct', nan, ...
            'coverageExecPct', nan, 'coverageChainPct', nan); %#ok<AGROW>
        continue;
    end

    hairAppState.hairLayer.haircutState.lengthGoal = regionPlan.lengthGoal;
    hairAppState.hairLayer.haircutState.params.targetHairLengthM = regionPlan.targetLengthM;
    hairAppState.hairLayer.haircutState.params.targetHairLengthToleranceM = regionPlan.lengthGoal.toleranceM;

    [executionState, firstStroke] = hairsim.execution.loadPlan(regionPlan);
    qJumpTarget = firstStroke.planResult.replayPositions(:, 1);

    if isCamClipper
        % camClipper strokes carry their own station; regions are always
        % entered through the generic safe connector
        targetTheta = firstStroke.railThetaRad;
        currentTheta = robotAppState.state.railThetaRad;
        if isfinite(targetTheta) && (~isfinite(currentTheta) || ...
                abs(angle(exp(1i * (targetTheta - currentTheta)))) > 1e-6)
            targetBasePlacementId = "rail" + round(rad2deg(targetTheta));
            [robotAppState, hairAppState, recordingState, numTicks, bundle, slideWarning] = foldSlideUnfold( ...
                robotAppState, hairAppState, recordingState, viewState, scene, floorZ, numTicks, bundle, ...
                robotAppState.state.qCurrent, previousBridgeLabel, targetBasePlacementId, options.RailSlideStepDeg);
            bridgeWarning = strtrim(bridgeWarning + " " + slideWarning);
            previousBridgeLabel = sprintf("Home@%s", targetBasePlacementId);
            bridgeFromHome = true;
        elseif ~isempty(previousLastStroke)
            bridgeFromHome = true;
        end
    end

    if bridgeFromHome
        unfoldLabel = sprintf("Unfold %s->%s", previousBridgeLabel, regionName);
        [robotAppState, hairAppState, recordingState, numTicks, bundle, unfoldWarning] = replaySafeTransition( ...
            robotAppState.state.qCurrent, qJumpTarget, unfoldLabel, ...
            robotAppState, hairAppState, recordingState, viewState, scene, floorZ, numTicks, bundle);
        bridgeWarning = strtrim(bridgeWarning + " " + unfoldWarning);
        bridgeFromHome = false;
    else
        robotAppState.state.qCommanded = qJumpTarget;
        robotAppState.state.qCurrent = qJumpTarget;
    end

    activeNodeCountsBeforeRegion = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;
    executionState = hairsim.execution.startRun(executionState);
    while executionState.isActive
        [executionState, poseCommand, info] = hairsim.execution.stepExecution(executionState, robotAppState.modelContext);
        if ~isempty(info.railStationSwitchRequest)
            robotAppState = hairsim.app.robot.applyRailStationForReplay(robotAppState, info.railStationSwitchRequest);
            continue;
        end
        if isempty(poseCommand)
            continue;
        end
        [robotAppState, hairAppState, recordingState, tickCounted, bundle] = applyAndRecordTick( ...
            poseCommand, robotAppState, hairAppState, recordingState, viewState, scene, floorZ, bundle);
        numTicks = numTicks + tickCounted;
        if poseCommand.cutFlag
            numCutTicks = numCutTicks + 1;
        end
    end
    activeNodeCountsAfterRegion = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;
    removedNodeCountRegion = sum(max(0, activeNodeCountsBeforeRegion(:) - activeNodeCountsAfterRegion(:)));

    regionResults(end + 1) = struct('regionName', regionName, 'executable', true, ...
        'message', regionPlan.message, 'numChainedStrokes', numel(regionPlan.chainStrokeIdx), ...
        'bridgeWarning', bridgeWarning, 'removedNodeCount', removedNodeCountRegion, ...
        'coverageScalpPct', 100 * regionPlan.coverage.scalp, 'coverageSafePct', 100 * regionPlan.coverage.safe, ...
        'coverageExecPct', 100 * regionPlan.coverage.exec, 'coverageChainPct', 100 * regionPlan.coverage.chain); %#ok<AGROW>
    previousLastStroke = regionPlan.strokes(regionPlan.chainStrokeIdx(end));
    previousBridgeLabel = regionName;
end

% Closing sequence: return to Home (same validated connector as every
% Home leg) and keep stepping hair/debris physics (q held at Home, no
% cutting) until the debris list is empty, so the demo doesn't just stop
% mid-stroke with the arm still out over the head and cut hair still
% airborne -- without this, the LAST cut's falling/landing debris would
% simply never be shown settling once execution stops advancing it.
if ~isempty(previousLastStroke)
    qLast = previousLastStroke.planResult.replayPositions(:, end);
    homeLabel = sprintf("Home %s->End", previousBridgeLabel);
    [robotAppState, hairAppState, recordingState, numTicks, bundle, homeWarning] = replaySafeTransition( ...
        qLast, robotAppState.modelContext.qHome, homeLabel, ...
        robotAppState, hairAppState, recordingState, viewState, scene, floorZ, numTicks, bundle);
    if strlength(homeWarning) > 0
        warning("hairsim:FullDemoReturnHomeInfeasible", "%s: %s", homeLabel, homeWarning);
    end

    maxSettleTicks = 300;
    settleTicks = 0;
    while ~isempty(hairAppState.hairLayer.haircutState.debrisList) && settleTicks < maxSettleTicks
        poseCommand = struct( ...
            'qCandidate', robotAppState.modelContext.qHome, 'solved', true, 'toolTipWorldM', zeros(1, 3), ...
            'statusLabel', sprintf("Settle:%d", settleTicks + 1), 'toolTipErrorM', 0, ...
            'targetTipWorldM', zeros(1, 3), 'desiredTipWorldM', zeros(1, 3), 'safeTipWorldM', zeros(1, 3), ...
            'governorActive', false, 'cutFlag', false);
        [robotAppState, hairAppState, recordingState, tickCounted, bundle] = applyAndRecordTick( ...
            poseCommand, robotAppState, hairAppState, recordingState, viewState, scene, floorZ, bundle);
        numTicks = numTicks + tickCounted;
        settleTicks = settleTicks + 1;
    end
end

recordingState = hairsim.app.recording.endSession(recordingState);
config = hairsim.control.createRobotHaircutConfig(scene.paths, recordingState.runPaths, ...
    'Mode', "auto-benchmark", 'ToolMount', robotAppState.modelContext.toolMount, ...
    'TargetHairLengthM', options.TargetLengthM, ...
    'TargetHairLengthToleranceM', hairAppState.hairLayer.haircutState.params.targetHairLengthToleranceM);
[recordingState, recordingResult] = hairsim.app.recording.finalizeSession(recordingState, ...
    robotAppState.modelContext, scene.paths, scene.headScene.headMeshWorld, ...
    initialActiveNodeCounts, hairAppState.hairLayer.haircutState, {}, config); %#ok<ASGLU>

bundle.qCandidate = [bundle.qCandidate{:}];
bundle.railThetaRad = cell2mat(bundle.railThetaRad);
bundle.basePlacementId = [bundle.basePlacementId{:}];
bundle.cutFlag = cell2mat(bundle.cutFlag);
bundle.statusLabel = [bundle.statusLabel{:}];
bundle.meta = struct('adapterName', options.AdapterName, 'dataDir', options.DataDir, ...
    'planningDataDir', options.PlanningDataDir, 'runId', options.RunId, ...
    'targetLengthM', options.TargetLengthM, 'plannerName', options.PlannerName);

if strlength(options.SaveBundlePath) > 0
    save(options.SaveBundlePath, 'bundle', '-v7.3');
end

% Stats: per-region removedNodeCount/coverage (regionResults, already
% enriched above) + the global hairsim.app.recording.finalizeSession
% metrics, written next to the rest of this run's output (same folder as
% metrics.json/summary.txt) as both a flat per-region CSV and one combined
% JSON (summary + per-region together).
statsDir = recordingState.runPaths.runDir;
writetable(struct2table(regionResults), fullfile(statsDir, 'demo_region_stats.csv'));
statsSummary = struct( ...
    'runId', options.RunId, ...
    'basePlacementMode', options.BasePlacementId, ...
    'regionOrder', options.RegionNames, ...
    'numTicks', numTicks, ...
    'numCutTicks', numCutTicks, ...
    'numRegionsWithWarning', nnz(arrayfun(@(r) strlength(r.bridgeWarning) > 0, regionResults)), ...
    'global', recordingResult.metrics, ...
    'perRegion', regionResults);
fid = fopen(fullfile(statsDir, 'demo_stats.json'), 'w');
fprintf(fid, '%s', jsonencode(statsSummary, 'PrettyPrint', true));
fclose(fid);

result = struct('regionResults', regionResults, 'recordingResult', recordingResult, ...
    'numTicks', numTicks, 'numCutTicks', numCutTicks, 'bundle', bundle, ...
    'hairAppState', hairAppState, 'scene', scene, 'statsDir', statsDir);
end

% =========================================================================

function stationDeg = regionRailStationDeg(regionName)
%REGIONRAILSTATIONDEG Frozen per-region best rail station from the
%   2026-09-25 matlab_ur5e rail-placement study (region-wise 50/50
%   executable, C_chain,rail ~60.2 %; stations 135/150/180). Not
%   re-derived here -- this is the study's own locked result, ported as
%   data only.
switch regionName
    case {"scalp_top_left", "scalp_left"}
        stationDeg = 135;
    case "scalp_right"
        stationDeg = 150;
    case {"scalp_back", "scalp_top_right"}
        stationDeg = 180;
    otherwise
        error("hairsim:UnknownRailRegion", ...
            "No frozen rail station for region '%s'.", regionName);
end
end

function ctx = buildSafeTransitionCtx(robotAppState, scene)
%BUILDSAFETRANSITIONCTX ctx for hairsim.planning.planSafeConfigurationTransition,
%   rebuilt from the CURRENT robotAppState.modelContext (it changes across a
%   rail-station switch): the arm, the arm + clipper body for self
%   collision, and the clipper model with the exact head mesh for clearance.
%   Reads scene.headScene (public), never robotAppState.headScene.
mc = robotAppState.modelContext;
tool = mc.toolMount.tool;
ctx = struct( ...
    'robot', mc.robot, ...
    'armWithToolRobot', hairsim.tool.createArmWithClipperBody(mc.robot, mc.endEffector, tool, mc.qHome), ...
    'endEffector', mc.endEffector, ...
    'tool', tool, ...
    'headQuery', hairsim.tool.createHeadDistanceQuery(scene.headScene.headMeshWorld), ...
    'headCenterWorldM', scene.headScene.headCenterWorldM, ...
    'collisionObjects', {mc.headCollision});
end

function [robotAppState, hairAppState, recordingState, numTicks, bundle, warningText] = replaySafeTransition( ...
        qa, qb, label, robotAppState, hairAppState, recordingState, viewState, scene, floorZ, numTicks, bundle)
%REPLAYSAFETRANSITION Call hairsim.planning.planSafeConfigurationTransition(qa,qb)
%   and, if feasible, replay its RETREAT/TRANSFER/APPROACH joint path tick
%   by tick; otherwise fall back to a teleport to qb and return a non-empty
%   warningText (the demo is not aborted).
warningText = "";
ctx = buildSafeTransitionCtx(robotAppState, scene);
transition = hairsim.planning.planSafeConfigurationTransition(qa, qb, ctx);
if transition.feasible
    for sampleIdx = 1:size(transition.Q, 2)
        poseCommand = struct( ...
            'qCandidate', transition.Q(:, sampleIdx), 'solved', true, 'toolTipWorldM', zeros(1, 3), ...
            'statusLabel', sprintf("%s:%s:%d/%d", label, transition.phase(sampleIdx), sampleIdx, size(transition.Q, 2)), ...
            'toolTipErrorM', 0, 'targetTipWorldM', zeros(1, 3), 'desiredTipWorldM', zeros(1, 3), ...
            'safeTipWorldM', zeros(1, 3), 'governorActive', false, 'cutFlag', false);
        [robotAppState, hairAppState, recordingState, tickCounted, bundle] = applyAndRecordTick( ...
            poseCommand, robotAppState, hairAppState, recordingState, viewState, scene, floorZ, bundle);
        numTicks = numTicks + tickCounted;
    end
else
    warningText = sprintf("safe transition infeasible (%s); fell back to a teleport", transition.reason);
    warning("hairsim:FullDemoSafeTransitionInfeasible", "%s: %s", label, warningText);
    robotAppState.state.qCommanded = qb;
    robotAppState.state.qCurrent = qb;
end
end

function thetaStepsRad = railSlideSteps(fromThetaRad, toThetaRad, stepRad)
%RAILSLIDESTEPS Intermediate thetas from fromThetaRad to toThetaRad
%   (fromThetaRad excluded, toThetaRad always last), each step <= stepRad.
%   The rail is a full ring: the slide takes the shorter way round, the
%   intermediate angles are not wrapped (the base transform is periodic).
delta = mod(toThetaRad - fromThetaRad + pi, 2 * pi) - pi;
numSteps = max(1, ceil(abs(delta) / stepRad));
t = (1:numSteps) / numSteps;
thetaStepsRad = fromThetaRad + delta * t;
thetaStepsRad(end) = toThetaRad;
end

function [robotAppState, hairAppState, recordingState, numTicks, bundle, warningText] = foldSlideUnfold( ...
        robotAppState, hairAppState, recordingState, viewState, scene, floorZ, numTicks, bundle, ...
        qBeforeFold, previousBridgeLabel, targetBasePlacementId, stepDeg)
%FOLDSLIDEUNFOLD Physically move the rail from the current station to
%   targetBasePlacementId's station: (1) connect qBeforeFold (the last
%   pose of the previous region's chain) to qHome at the CURRENT station
%   via hairsim.planning.planSafeConfigurationTransition -- the same
%   generic, region/lane-agnostic retreat -> joint-line/shell-arc ->
%   approach connector, (2) animate the rail
%   sliding station to station with q held at qHome
%   (hairsim.app.robot.applyRailStationForReplay rebuilds the robot model
%   at each intermediate theta; a live arm self-collision + robot-vs-head
%   check runs every step as a sanity check on the study's own "qHome is
%   collision-free across the swept range" finding, not blind trust), (3)
%   hairsim.app.robot.switchBase finalizes the new station officially.
%   warningText is non-empty if the fold connection was infeasible (falls
%   back to a teleport to qHome) or a slide step failed its sanity check
%   (the slide still proceeds -- this is a flag for review, not a hard
%   stop, same policy as the Home-to-stroke connection's own infeasible
%   case in the main loop).
foldLabel = sprintf("Fold %s->Home", previousBridgeLabel);
[robotAppState, hairAppState, recordingState, numTicks, bundle, warningText] = replaySafeTransition( ...
    qBeforeFold, robotAppState.modelContext.qHome, foldLabel, ...
    robotAppState, hairAppState, recordingState, viewState, scene, floorZ, numTicks, bundle);

fromThetaRad = robotAppState.state.railThetaRad;   % current even after in-plan rail switches
if ~isfinite(fromThetaRad)
    fromThetaRad = deg2rad(str2double(extractAfter(robotAppState.state.basePlacementId, "rail")));
end
toThetaRad = deg2rad(str2double(extractAfter(targetBasePlacementId, "rail")));
thetaStepsRad = railSlideSteps(fromThetaRad, toThetaRad, deg2rad(stepDeg));
qHome = robotAppState.modelContext.qHome;
for stepIdx = 1:numel(thetaStepsRad)
    robotAppState = hairsim.app.robot.applyRailStationForReplay(robotAppState, thetaStepsRad(stepIdx));
    armOnlyRobot = robotAppState.modelContext.robot;
    unsafe = checkCollision(armOnlyRobot, qHome, 'SkippedSelfCollisions', 'parent');
    reason = "SELF_COLLISION";
    if ~unsafe
        [clear_, reason, ~] = hairsim.motion.validateClipperRobotCollision( ...
            robotAppState.modelContext.robot, qHome, robotAppState.modelContext.endEffector, ...
            robotAppState.modelContext.headCollision, 0.001, true);
        unsafe = ~clear_;
    end
    if unsafe && strlength(warningText) == 0
        warningText = sprintf("rail slide step %d/%d unsafe at qHome (%s)", ...
            stepIdx, numel(thetaStepsRad), reason);
        warning("hairsim:FullDemoRailSlideUnsafe", "%s", warningText);
    end
    poseCommand = struct( ...
        'qCandidate', qHome, 'solved', true, 'toolTipWorldM', zeros(1, 3), ...
        'statusLabel', sprintf("RailSlide:%.0fdeg:%d/%d", rad2deg(thetaStepsRad(stepIdx)), stepIdx, numel(thetaStepsRad)), ...
        'toolTipErrorM', 0, 'targetTipWorldM', zeros(1, 3), 'desiredTipWorldM', zeros(1, 3), ...
        'safeTipWorldM', zeros(1, 3), 'governorActive', false, 'cutFlag', false);
    [robotAppState, hairAppState, recordingState, tickCounted, bundle] = applyAndRecordTick( ...
        poseCommand, robotAppState, hairAppState, recordingState, viewState, scene, floorZ, bundle);
    numTicks = numTicks + tickCounted;
end

robotAppState = hairsim.app.robot.switchBase(robotAppState, targetBasePlacementId);
robotAppState = applyHomeOverride(robotAppState);
end

function robotAppState = applyHomeOverride(robotAppState)
% camClipper: switchBase re-installs the adapter's Home; restore the
% clipper-safe park pose (the arm was held there during the slide).
if isfield(robotAppState, 'homeOverride') && ~isempty(robotAppState.homeOverride)
    robotAppState.modelContext.qHome = robotAppState.homeOverride(:);
    robotAppState.state.qCommanded = robotAppState.homeOverride(:);
    robotAppState.state.qCurrent = robotAppState.homeOverride(:);
end
end

function [robotAppState, hairAppState, recordingState, tickCounted, bundle] = applyAndRecordTick( ...
        poseCommand, robotAppState, hairAppState, recordingState, viewState, scene, floorZ, bundle)
%APPLYANDRECORDTICK Shared per-sample body for a normal stroke/transit
%   replay sample (from hairsim.execution.stepExecution), a cross-region
%   bridge sample, or a rail-slide sample (both from
%   hairsim.planning.planSafeConfigurationTransition or a fixed qHome,
%   called directly in runFullDemoSession): apply the pose, step hair, append a
%   recording sample, render, AND append this sample to the flat replay
%   bundle (see runFullDemoSession's own header) -- this is the ONE choke
%   point every tick of the whole demo passes through, so it is also the
%   one place the bundle needs to be filled in.
lengthGoalBeforeTick = hairAppState.hairLayer.haircutState.lengthGoal;
[robotAppState, hairAppState, recordingState, tickCounted] = hairsim.app.applyPoseTick( ...
    poseCommand, robotAppState, hairAppState, recordingState, viewState, scene, floorZ);
bundle.qCandidate{end + 1} = poseCommand.qCandidate(:);
bundle.railThetaRad{end + 1} = robotAppState.state.railThetaRad;
bundle.basePlacementId{end + 1} = robotAppState.state.basePlacementId;
bundle.cutFlag{end + 1} = poseCommand.cutFlag;
bundle.statusLabel{end + 1} = string(poseCommand.statusLabel);
bundle.lengthGoal{end + 1} = lengthGoalBeforeTick;
end
