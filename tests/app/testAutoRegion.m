function result = testAutoRegion(options)
%TESTAUTOREGION Headless Auto, end to end: plan one region with the
%   release planner and run the plan through the execution engine and the
%   shared per-tick body, exactly as hairsim.app.runAutoSession's Run does
%   (comb-lift cut model, rail station switches), with no figure. Checks:
%     - the plan is executable and its chain covers every root the robot
%       can reach (C_B, from the planner's own evaluation B);
%     - the strands the run cut in the region are EXACTLY the ones
%       evaluation B predicted (plan coverage == executed cut);
%     - every cut strand ends at its target node (no over- or under-cut).
%   Strands of neighbouring regions may be cut too (the toolpaths overlap
%   the border) and are only held to the target rule.

arguments
    options.RegionName (1, 1) string = "scalp_back"
    options.TargetLengthM (1, 1) double = 0.005
    options.BasePlacementId (1, 1) string = "rail180"
    options.PlannerName (1, 1) string = "camClipperAgainstFlow"
    options.MinCoverage (1, 1) double = 1.0      % required C_B
end

scene = hairsim.api.loadScene();
robotAppState = hairsim.app.robot.createRobotAppState("ur5e", scene);
if options.BasePlacementId ~= "fixed"
    robotAppState = hairsim.app.robot.switchBase(robotAppState, options.BasePlacementId);
end
hairAppState = hairsim.app.hair.createHairAppState(scene);
floorZ = scene.headScene.headMeshWorld.bboxMin(3) - 0.35 * 0.5 * max(scene.headScene.headMeshWorld.bboxSize);
countsBefore = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;

% ---- plan (the same request the Auto entry builds; no cache)
planRequest = struct('robotModel', robotAppState.modelContext, 'qCurrent', robotAppState.state.qCurrent, ...
    'headMeshWorld', scene.headScene.headMeshWorld, 'headMeshSafetyWorld', scene.headScene.headMeshSafetyWorld, ...
    'headCenterWorldM', scene.headScene.headCenterWorldM, 'rWorldHead', scene.headScene.R_world_head, ...
    'basePlacementId', robotAppState.state.basePlacementId, ...
    'haircutState', hairAppState.hairLayer.haircutState, 'regionName', options.RegionName, ...
    'targetLengthM', options.TargetLengthM, 'plannerName', options.PlannerName, ...
    'options', struct('PlanTransits', true, 'OnProgress', @(m) fprintf('%s  %s\n', datestr(now, 'HH:MM:SS'), m)), ...
    'useCache', false);
planningState = hairsim.app.planning.createAutoPlanningState(tempname);
[~, planResult] = hairsim.app.planning.planHaircut(planningState, planRequest);
regionPlan = planResult.regionPlan;
cmp = regionPlan.camClipper.comparison;
fprintf('plan: %d/%d strokes chained, C_A %.1f%%, C_B %.1f%%\n', numel(regionPlan.chainStrokeIdx), ...
    regionPlan.numStrokes, 100 * cmp.C_A, 100 * cmp.C_B);
assert(regionPlan.executable, "plan not executable: %s", regionPlan.message);
assert(cmp.C_B >= options.MinCoverage - 1e-12, "C_B %.4f below %.4f", cmp.C_B, options.MinCoverage);

% ---- run it, as runAutoSession's onRun + tickExecution do
[executionState, firstStroke] = hairsim.execution.loadPlan(regionPlan);
hairAppState.hairLayer.cutting.model = hairsim.hair.cutting.createCombLiftCutModelForPlanner( ...
    regionPlan.plannerName, hairAppState.hairLayer.haircutState, scene.headScene.headMeshWorld);
hairAppState.hairLayer.haircutState.lengthGoal = regionPlan.lengthGoal;
if isfinite(firstStroke.railThetaRad)
    robotAppState = hairsim.app.robot.applyRailStationForReplay(robotAppState, firstStroke.railThetaRad);
end
robotAppState.state.qCommanded = firstStroke.planResult.replayPositions(:, 1);
robotAppState.state.qCurrent = firstStroke.planResult.replayPositions(:, 1);
executionState = hairsim.execution.startRun(executionState);
numTicks = 0;
while executionState.isActive
    [executionState, poseCommand, info] = hairsim.execution.stepExecution(executionState, robotAppState.modelContext);
    if ~isempty(info.railStationSwitchRequest)
        robotAppState = hairsim.app.robot.applyRailStationForReplay(robotAppState, info.railStationSwitchRequest);
        continue;
    end
    if isempty(poseCommand)
        continue;
    end
    [robotAppState, hairAppState] = hairsim.app.applyPoseTick(poseCommand, robotAppState, hairAppState, ...
        struct(), [], scene, floorZ, 'Record', false);
    numTicks = numTicks + 1;
end

% ---- executed cut vs evaluation B
counts = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;
target = regionPlan.lengthGoal.targetActiveNodeCounts;
cut = find(counts < countsBefore);
surface = regionPlan.camClipper.toolpathPlan.surface;
needIdx = surface.strandIdx(surface.needsCut);
predicted = sort(needIdx(regionPlan.targetCovered));
regionCut = sort(intersect(cut, surface.strandIdx));
fprintf(['run: %d ticks, cut %d strands: %d in %s (evaluation B predicted %d, identical: %d), ', ...
    '%d in neighbouring regions\n'], numTicks, numel(cut), numel(regionCut), options.RegionName, ...
    numel(predicted), isequal(regionCut(:), predicted(:)), numel(cut) - numel(regionCut));
assert(all(counts(cut) == target(cut)), "a strand was cut somewhere other than its target node");
assert(isequal(regionCut(:), predicted(:)), "executed region cut differs from evaluation B");
fprintf('PASS: Auto %s (%s, %s, %.1f cm)\n', options.RegionName, options.PlannerName, ...
    options.BasePlacementId, 100 * options.TargetLengthM);
result = struct('regionPlan', regionPlan, 'cutStrands', cut, 'numTicks', numTicks);
end
