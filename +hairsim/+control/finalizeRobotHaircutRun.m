function result = finalizeRobotHaircutRun(record)
%FINALIZEROBOTHAIRCUTRUN Write robot haircut outputs and summary metrics.

runPaths = record.runPaths;
trajectory = hairsim.control.writeClipperTrajectory( ...
    record.trajectoryRecorder, runPaths.clipperTrajectoryCsv);
hairsim.control.writeClipperTrajectory(record.trajectoryRecorder, record.paths.clipperTrajectoryCsv);

writeRobotJointTrajectoryCsv( ...
    runPaths.robotJointTrajectoryCsv, record.sampleLog.time, record.sampleLog.q);
writeUr5eCartesianCsv(record.robot, record.sampleLog.q', ...
    runPaths.ur5eCartesianCsv, char(record.endEffector), 'XYZ', false);

meshClearance = hairsim.evaluateClipperSphereMeshClearance( ...
    record.sampleLog.clipperCenterWorldM, record.headMeshSafetyWorld, ...
    hairsim.getToolClearanceRadiusM(record.toolMount));

poseLog = createPoseLogTable(record.sampleLog, meshClearance);
writetable(poseLog, runPaths.poseLogCsv);

cutEventTable = createCutEventTable(record.cutEvents);
writetable(cutEventTable, runPaths.cutEventsCsv);

writeAutoPlanOutputs(runPaths, record.robot, record.endEffector, record.autoPlanHistory);

metrics = computeRobotHaircutMetrics(record.initialActiveNodeCounts, ...
    record.finalHaircutState, trajectory, poseLog, cutEventTable, ...
    meshClearance, record.autoPlanHistory);
hairsim.control.writeJsonFile(runPaths.metricsJson, metrics);
summary = createRobotHaircutSummary(runPaths, record.config, metrics);
hairsim.control.writeTextFile(runPaths.summaryTxt, summary);

result = struct();
result.paths = runPaths;
result.config = record.config;
result.metrics = metrics;
result.trajectory = trajectory;
result.poseLog = poseLog;
result.cutEvents = cutEventTable;
result.autoPlanHistory = record.autoPlanHistory;
fprintf('%s', summary);
fprintf('Saved compatibility clipper trajectory: %s\n', record.paths.clipperTrajectoryCsv);
end

function writeRobotJointTrajectoryCsv(filePath, sampleTimes, qSamples)
jointColumns = cell(1, size(qSamples, 2));
for jointIdx = 1:numel(jointColumns)
    jointColumns{jointIdx} = sprintf('q%d_rad', jointIdx);
end

if isempty(qSamples)
    trajectoryTable = array2table(zeros(0, numel(jointColumns) + 1), ...
        'VariableNames', [{'time'}, jointColumns]);
else
    trajectoryTable = array2table([sampleTimes(:), qSamples], ...
        'VariableNames', [{'time'}, jointColumns]);
end
writetable(trajectoryTable, filePath);
end

function writeUr5eCartesianCsv(robot, qTrajectory, filePath, endEffector, eulerSeq, addHeader)
if isempty(qTrajectory)
    cartesianTable = table(zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
        zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
        'VariableNames', {'cartesian_pose.x', 'cartesian_pose.y', ...
        'cartesian_pose.z', 'cartesian_pose.theta_x', ...
        'cartesian_pose.theta_y', 'cartesian_pose.theta_z'});
else
    numSamples = size(qTrajectory, 2);
    xyz = zeros(numSamples, 3);
    rpyDeg = zeros(numSamples, 3);
    for sampleIdx = 1:numSamples
        T_world_ee = getTransform(robot, qTrajectory(:, sampleIdx), endEffector);
        xyz(sampleIdx, :) = T_world_ee(1:3, 4)';
        rpyDeg(sampleIdx, :) = rad2deg(rotm2eul(T_world_ee(1:3, 1:3), eulerSeq));
    end

    cartesianTable = table( ...
        xyz(:, 1), xyz(:, 2), xyz(:, 3), ...
        rpyDeg(:, 1), rpyDeg(:, 2), rpyDeg(:, 3), ...
        'VariableNames', {'cartesian_pose.x', 'cartesian_pose.y', ...
        'cartesian_pose.z', 'cartesian_pose.theta_x', ...
        'cartesian_pose.theta_y', 'cartesian_pose.theta_z'});
end

if addHeader
    fid = fopen(filePath, 'w');
    if fid < 0
        error("hairsim:FileOpenFailed", ...
            "Could not open UR5e CSV for writing: %s", filePath);
    end
    cleanupObj = onCleanup(@() fclose(fid));
    fprintf(fid, '### starts from row 2 -> row 2 = position 1 etc. ###\n');
    clear cleanupObj;
    writetable(cartesianTable, filePath, 'WriteMode', 'append');
else
    writetable(cartesianTable, filePath);
end
end

function poseLog = createPoseLogTable(sampleLog, meshClearance)
poseLog = table( ...
    sampleLog.time(:), sampleLog.mode(:), sampleLog.status(:), ...
    logical(sampleLog.solved(:)), logical(sampleLog.cutFlag(:)), ...
    logical(sampleLog.governorActive(:)), sampleLog.toolTipErrorM(:), ...
    sampleLog.cutCount(:), meshClearance.centerClearanceM(:), ...
    meshClearance.surfaceClearanceM(:), ...
    sampleLog.targetTipWorldM(:, 1), sampleLog.targetTipWorldM(:, 2), sampleLog.targetTipWorldM(:, 3), ...
    sampleLog.desiredTipWorldM(:, 1), sampleLog.desiredTipWorldM(:, 2), sampleLog.desiredTipWorldM(:, 3), ...
    sampleLog.safeTipWorldM(:, 1), sampleLog.safeTipWorldM(:, 2), sampleLog.safeTipWorldM(:, 3), ...
    sampleLog.actualTipWorldM(:, 1), sampleLog.actualTipWorldM(:, 2), sampleLog.actualTipWorldM(:, 3), ...
    sampleLog.clipperCenterWorldM(:, 1), sampleLog.clipperCenterWorldM(:, 2), sampleLog.clipperCenterWorldM(:, 3), ...
    'VariableNames', {'time', 'mode', 'status', 'solved', 'cutFlag', ...
    'governorActive', 'toolTipErrorM', 'cutCount', 'centerClearanceM', ...
    'surfaceClearanceM', 'targetTipX', 'targetTipY', 'targetTipZ', ...
    'desiredTipX', 'desiredTipY', 'desiredTipZ', ...
    'safeTipX', 'safeTipY', 'safeTipZ', ...
    'actualTipX', 'actualTipY', 'actualTipZ', ...
    'clipperCenterX', 'clipperCenterY', 'clipperCenterZ'});
end

function cutEventTable = createCutEventTable(cutEvents)
if isempty(cutEvents.sampleIndex)
    cutEventTable = table(zeros(0, 1), zeros(0, 1), strings(0, 1), ...
        zeros(0, 1), zeros(0, 1), ...
        'VariableNames', {'sampleIndex', 'time', 'mode', 'strandIndex', 'nodeIndex'});
    return;
end

cutEventTable = table( ...
    cutEvents.sampleIndex(:), cutEvents.time(:), cutEvents.mode(:), ...
    cutEvents.strandIndex(:), cutEvents.nodeIndex(:), ...
    'VariableNames', {'sampleIndex', 'time', 'mode', 'strandIndex', 'nodeIndex'});
end

function writeAutoPlanOutputs(runPaths, robot, endEffector, autoPlanHistory)
if isempty(autoPlanHistory)
    emptyTable = table(zeros(0, 1), strings(0, 1), strings(0, 1), ...
        zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
        zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), false(0, 1), ...
        strings(0, 1), strings(0, 1), ...
        'VariableNames', {'planIndex', 'status', 'failureReason', ...
        'lastExitFlag', 'plannerSampleCount', 'targetTipErrorM', ...
        'eePositionErrorM', 'eeOrientationErrorRad', 'clipperCenterErrorM', ...
        'minSurfaceClearanceM', 'headPenetrationSampleCount', ...
        'hasHeadPenetration', 'jointTrajectoryCsv', 'ur5eCartesianCsv'});
    writetable(emptyTable, runPaths.autoPlanSummaryCsv);
    return;
end

summaryTable = table();
for planIdx = 1:numel(autoPlanHistory)
    planSummary = autoPlanHistory{planIdx};
    jointCsv = "";
    cartesianCsv = "";
    if isfield(planSummary, 'qTrajectory') && ~isempty(planSummary.qTrajectory)
        jointCsv = string(fullfile(runPaths.runDir, ...
            sprintf('auto_plan_%02d_joint_trajectory.csv', planIdx)));
        cartesianCsv = string(fullfile(runPaths.runDir, ...
            sprintf('auto_plan_%02d_ur5e_cartesian_trajectory.csv', planIdx)));
        writeRobotJointTrajectoryCsv(char(jointCsv), ...
            planSummary.timestamp(:), planSummary.qTrajectory');
        writeUr5eCartesianCsv(robot, planSummary.qTrajectory, ...
            char(cartesianCsv), char(endEffector), 'XYZ', false);
    end

    row = table( ...
        planIdx, string(planSummary.status), string(planSummary.failureReason), ...
        planSummary.lastExitFlag, planSummary.plannerSampleCount, ...
        planSummary.targetTipErrorM, planSummary.eePositionErrorM, ...
        planSummary.eeOrientationErrorRad, planSummary.clipperCenterErrorM, ...
        planSummary.minSurfaceClearanceM, planSummary.headPenetrationSampleCount, ...
        logical(planSummary.hasHeadPenetration), jointCsv, cartesianCsv, ...
        'VariableNames', {'planIndex', 'status', 'failureReason', ...
        'lastExitFlag', 'plannerSampleCount', 'targetTipErrorM', ...
        'eePositionErrorM', 'eeOrientationErrorRad', 'clipperCenterErrorM', ...
        'minSurfaceClearanceM', 'headPenetrationSampleCount', ...
        'hasHeadPenetration', 'jointTrajectoryCsv', 'ur5eCartesianCsv'});
    summaryTable = [summaryTable; row]; %#ok<AGROW>
end
writetable(summaryTable, runPaths.autoPlanSummaryCsv);
end

function metrics = computeRobotHaircutMetrics( ...
        initialActiveNodeCounts, finalHaircutState, trajectory, poseLog, ...
        cutEventTable, meshClearance, autoPlanHistory)
finalActiveNodeCounts = finalHaircutState.hairState.activeNodeCounts;
removedNodes = max(0, initialActiveNodeCounts(:) - finalActiveNodeCounts(:));
initialNodeCount = sum(initialActiveNodeCounts(:));

metrics = struct();
metrics.trajectorySampleCount = height(trajectory);
metrics.robotJointSampleCount = height(poseLog);
metrics.durationS = max(trajectory.time) - min(trajectory.time);
metrics.cutTrajectorySampleCount = double(sum(trajectory.cutFlag));
metrics.totalCutEvents = double(height(cutEventTable));
metrics.cutEventTableCount = double(height(cutEventTable));
metrics.cutStrandCount = double(nnz(removedNodes));
metrics.removedNodeCount = double(sum(removedNodes));
metrics.removedRatio = safeDivide(metrics.removedNodeCount, initialNodeCount);
metrics.remainingActiveNodeMean = double(mean(finalActiveNodeCounts));
metrics.remainingActiveNodeMin = double(min(finalActiveNodeCounts));
metrics.clipperTrajectoryLengthM = computeTrajectoryLengthM( ...
    [trajectory.x, trajectory.y, trajectory.z]);
metrics.poseAcceptedCount = double(nnz(poseLog.solved));
metrics.poseRejectedCount = double(height(poseLog) - nnz(poseLog.solved));
metrics.poseAcceptanceRate = safeDivide(metrics.poseAcceptedCount, height(poseLog));
metrics.governorActivationCount = double(nnz(poseLog.governorActive));
metrics.governorActivationRate = safeDivide(metrics.governorActivationCount, height(poseLog));
metrics.meanToolTipErrorM = meanOrNan(poseLog.toolTipErrorM);
metrics.maxToolTipErrorM = maxOrNan(poseLog.toolTipErrorM);
metrics.minCenterClearanceM = meshClearance.minCenterClearanceM;
metrics.minSurfaceClearanceM = meshClearance.minSurfaceClearanceM;
metrics.minHeadClearanceM = metrics.minSurfaceClearanceM;
metrics.headPenetrationSampleCount = meshClearance.headPenetrationSampleCount;
metrics.hasHeadPenetration = meshClearance.hasHeadPenetration;
metrics.autoPlanCount = double(numel(autoPlanHistory));
metrics.autoMeanTargetTipErrorM = meanAutoField(autoPlanHistory, 'targetTipErrorM');
metrics.autoMaxTargetTipErrorM = maxAutoField(autoPlanHistory, 'targetTipErrorM');
metrics.autoMinSurfaceClearanceM = minAutoField(autoPlanHistory, 'minSurfaceClearanceM');
metrics.autoHeadPenetrationSampleCount = sumAutoField(autoPlanHistory, ...
    'headPenetrationSampleCount');
metrics = appendUniformTargetLengthMetrics(metrics, finalHaircutState);
end

function lengthM = computeTrajectoryLengthM(points)
if size(points, 1) < 2
    lengthM = 0;
else
    lengthM = double(sum(vecnorm(diff(points, 1, 1), 2, 2)));
end
end

function value = safeDivide(numerator, denominator)
if denominator <= 0
    value = nan;
else
    value = double(numerator) / double(denominator);
end
end

function value = meanOrNan(values)
values = values(isfinite(values));
if isempty(values)
    value = nan;
else
    value = double(mean(values));
end
end

function value = maxOrNan(values)
values = values(isfinite(values));
if isempty(values)
    value = nan;
else
    value = double(max(values));
end
end

function value = meanAutoField(autoPlanHistory, fieldName)
values = collectAutoField(autoPlanHistory, fieldName, true);
value = meanOrNan(values);
end

function value = maxAutoField(autoPlanHistory, fieldName)
values = collectAutoField(autoPlanHistory, fieldName, true);
value = maxOrNan(values);
end

function value = minAutoField(autoPlanHistory, fieldName)
values = collectAutoField(autoPlanHistory, fieldName, true);
values = values(isfinite(values));
if isempty(values)
    value = nan;
else
    value = double(min(values));
end
end

function value = sumAutoField(autoPlanHistory, fieldName)
values = collectAutoField(autoPlanHistory, fieldName, false);
values = values(isfinite(values));
if isempty(values)
    value = 0;
else
    value = double(sum(values));
end
end

function metrics = appendUniformTargetLengthMetrics(metrics, finalHaircutState)
if ~isfield(finalHaircutState, 'lengthGoal') || ...
        finalHaircutState.lengthGoal.type ~= "uniform_length"
    return;
end

goal = finalHaircutState.lengthGoal;
remainingLengthsM = restRemainingLengths(goal, finalHaircutState.hairState);
targetPerStrandM = repmat(goal.targetLengthM, numel(remainingLengthsM), 1);
if isfield(goal, 'targetLengthPerStrandM')
    targetPerStrandM = goal.targetLengthPerStrandM(:);   % per-region goals
end
lengthErrorM = remainingLengthsM(:) - targetPerStrandM;
toleranceM = goal.toleranceM;
initialRemainingLengthsM = getGoalInitialRemainingLengths(goal, remainingLengthsM);
eligibleMask = initialRemainingLengthsM(:) >= targetPerStrandM;

metrics.targetHairLengthM = goal.targetLengthM;
metrics.targetHairLengthToleranceM = toleranceM;
metrics.targetLengthEligibleStrandCount = double(nnz(eligibleMask));
metrics.targetLengthInitiallyShortCount = double(nnz(~eligibleMask));
metrics.meanRemainingHairLengthM = meanOrNan(remainingLengthsM(:));
metrics.targetLengthMeanAbsErrorM = meanOrNan(abs(lengthErrorM));
metrics.targetLengthRmseM = sqrt(meanOrNan(lengthErrorM .^ 2));
metrics.targetLengthMaxAbsErrorM = maxOrNan(abs(lengthErrorM));
metrics.targetLengthOvercutCount = double(nnz( ...
    eligibleMask & lengthErrorM < -toleranceM));
metrics.targetLengthUndercutCount = double(nnz(lengthErrorM > toleranceM));
metrics.targetLengthWithinToleranceRate = safeDivide( ...
    nnz(abs(lengthErrorM) <= toleranceM), numel(lengthErrorM));
metrics.targetActiveNodeMean = meanOrNan( ...
    finalHaircutState.hairState.activeNodeCounts(:));
metrics.targetActiveNodeGoalMean = meanOrNan(goal.targetActiveNodeCounts(:));
end

function lengthsM = restRemainingLengths(goal, hairState)
% Remaining length measured along the REST shape (the goal's reference
% cumulative lengths at each strand's final active node count). The live
% polyline is deformed by the physics, so its length is not the haircut
% length (it once gave a 6.6 cm error on 3.6 cm hair).
if ~isfield(goal, 'referenceCumulativeLengthsM')
    lengthsM = hairsim.hair.computeActiveStrandLengths(hairState);
    return;
end
cumulative = goal.referenceCumulativeLengthsM;
counts = min(max(round(hairState.activeNodeCounts), 1), size(cumulative, 1));
lengthsM = cumulative(sub2ind(size(cumulative), counts, 1:numel(counts)));
end

function initialRemainingLengthsM = getGoalInitialRemainingLengths(goal, fallbackLengthsM)
if isfield(goal, 'initialRemainingLengthM')
    initialRemainingLengthsM = goal.initialRemainingLengthM;
else
    initialRemainingLengthsM = fallbackLengthsM;
end
end

function values = collectAutoField(autoPlanHistory, fieldName, successfulOnly)
values = nan(numel(autoPlanHistory), 1);
for planIdx = 1:numel(autoPlanHistory)
    planSummary = autoPlanHistory{planIdx};
    if successfulOnly && (~isfield(planSummary, 'success') || ~planSummary.success)
        continue;
    end
    if isfield(planSummary, fieldName)
        values(planIdx) = double(planSummary.(fieldName));
    end
end
end

function summary = createRobotHaircutSummary(runPaths, config, metrics)
summary = sprintf([ ...
    'HairSim robot haircut run: %s\n', ...
    'Stage: %s\n', ...
    'Mode: %s\n', ...
    'Output directory: %s\n', ...
    'Clipper trajectory samples: %d\n', ...
    'Robot joint samples: %d\n', ...
    'Accepted/rejected poses: %d/%d\n', ...
    'Governor activations: %d\n', ...
    'Total cut events: %d\n', ...
    'Cut strands: %d\n', ...
    'Removed nodes: %d\n', ...
    'Removed ratio: %.4f\n', ...
    'Clipper trajectory length: %.4f m\n', ...
    'Minimum surface clearance: %.4f m\n', ...
    'Head penetration samples: %d\n', ...
    'Auto mean target-tip error: %.4f m\n', ...
    'Config: %s\n', ...
    'Metrics: %s\n', ...
    'Pose log: %s\n', ...
    'Cut events: %s\n', ...
    'Clipper trajectory: %s\n', ...
    'Robot joint trajectory: %s\n', ...
    'UR5e Cartesian trajectory: %s\n'], ...
    runPaths.runId, config.stage, string(config.mode), runPaths.runDir, ...
    metrics.trajectorySampleCount, metrics.robotJointSampleCount, ...
    metrics.poseAcceptedCount, metrics.poseRejectedCount, ...
    metrics.governorActivationCount, metrics.totalCutEvents, ...
    metrics.cutStrandCount, metrics.removedNodeCount, metrics.removedRatio, ...
    metrics.clipperTrajectoryLengthM, metrics.minSurfaceClearanceM, ...
    metrics.headPenetrationSampleCount, metrics.autoMeanTargetTipErrorM, ...
    runPaths.configJson, runPaths.metricsJson, runPaths.poseLogCsv, ...
    runPaths.cutEventsCsv, runPaths.clipperTrajectoryCsv, ...
    runPaths.robotJointTrajectoryCsv, runPaths.ur5eCartesianCsv);
if isfield(metrics, 'targetHairLengthM')
    summary = [summary, sprintf([ ...
        'Target hair length: %.4f m\n', ...
        'Target length MAE/RMSE: %.4f/%.4f m\n', ...
        'Target length within tolerance: %.3f\n'], ...
        metrics.targetHairLengthM, metrics.targetLengthMeanAbsErrorM, ...
        metrics.targetLengthRmseM, metrics.targetLengthWithinToleranceRate)];
end
end
