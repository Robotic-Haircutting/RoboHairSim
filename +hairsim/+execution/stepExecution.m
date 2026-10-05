function [executionState, poseCommand, info] = stepExecution(executionState, robotModelContext)
%STEPEXECUTION Advance the current segment by one sample. Ported from
%   run_robot_haircut.m's runAutoControlStep + advanceRegionRun (auto-
%   advancing to the next queued segment, or finishing the run, happens
%   inside this function -- Execution owns that bookkeeping).
%
%   robotModelContext (read-only, explicit input -- Execution never holds
%   its own reference to the robot): robot, endEffector, toolMount. Used
%   only for forward kinematics (to compute toolTipWorldM/targetTipWorldM/
%   toolTipErrorM for the returned PoseCommand).
%
%   poseCommand: the usual PoseCommand shape (qCandidate, solved,
%   toolTipWorldM, statusLabel, toolTipErrorM, targetTipWorldM,
%   desiredTipWorldM, safeTipWorldM, governorActive, cutFlag) -- hand it to
%   hairsim.app.robot.applyPoseCommand exactly like a Teleoperation
%   PoseCommand. desiredTipWorldM/safeTipWorldM both equal targetTipWorldM
%   and governorActive is always false (replay never runs the teleop
%   safety governor -- the plan was already validated when it was made).
%
%   cutFlag in poseCommand is this sample's OWN recorded cut flag from the
%   plan; whether to actually apply it to Hair/Sim is still the caller's
%   call (an idle/paused session should not cut even during a replayed CUT
%   sample -- that gating is session-lifecycle, not Execution's concern).
%
%   info fields: active (false if nothing is loaded/running),
%   railStationSwitchRequest (double thetaRad, or [] -- see below),
%   segmentJustCompleted, runComplete, nextSegmentInfo (only when a new
%   segment was just auto-loaded; see hairsim.execution.startRun's
%   segmentInfo shape), strokesDone.
%
%   RAIL-SLIDE MID-SEGMENT SWITCH (two-call protocol): if the current
%   sample is the one at which a rail-slide transit's station actually
%   changes, this function does NOT compute a pose -- it returns
%   poseCommand = [] and info.railStationSwitchRequest = thetaRad instead.
%   The caller must apply hairsim.app.robot.applyRailStationForReplay(
%   robotAppState, thetaRad) and call stepExecution again (same tick,
%   trajectoryIndex unchanged) with the updated robotModelContext to get
%   the real PoseCommand. This keeps Execution from ever touching
%   RobotModelContext itself.

arguments
    executionState (1, 1) struct
    robotModelContext (1, 1) struct
end

info = struct('active', false, 'railStationSwitchRequest', [], ...
    'segmentJustCompleted', false, 'runComplete', false, 'nextSegmentInfo', [], ...
    'strokesDone', executionState.strokesDone);
poseCommand = [];
if executionState.trajectoryIndex <= 0 || isempty(executionState.currentSegment)
    return;
end
info.active = true;

seg = executionState.currentSegment;
idx = executionState.trajectoryIndex;
numSamples = size(seg.replayPositions, 2);

% A transit may carry several station switches (switchIdx / toThetaRad
% vectors, e.g. an animated rail slide: one switch per rail step with q
% held); they are served in order, one per call, same two-call protocol.
if seg.pendingStationSwitch.switchIdx(1) > 0 && idx >= seg.pendingStationSwitch.switchIdx(1)
    info.railStationSwitchRequest = seg.pendingStationSwitch.toThetaRad(1);
    if numel(seg.pendingStationSwitch.switchIdx) > 1
        executionState.currentSegment.pendingStationSwitch.switchIdx(1) = [];
        executionState.currentSegment.pendingStationSwitch.toThetaRad(1) = [];
    else
        executionState.currentSegment.pendingStationSwitch.switchIdx = 0;
    end
    return;
end

qCandidate = seg.replayPositions(:, idx);
solved = all(isfinite(qCandidate)) && seg.strokePlan.plan.status ~= "failed";
cutFlag = seg.replayCutFlags(idx);

T_world_ee_actual = getTransform(robotModelContext.robot, qCandidate, char(robotModelContext.endEffector));
actualTipWorldM = hairsim.getMountedClipperPoint(T_world_ee_actual, robotModelContext.toolMount.T_ee_tool);
targetTipWorldM = actualTipWorldM;
if isfield(seg.strokePlan, 'replayTipTargetsWorldM') && idx <= size(seg.strokePlan.replayTipTargetsWorldM, 1)
    targetTipWorldM = seg.strokePlan.replayTipTargetsWorldM(idx, :);
end
toolTipErrorM = norm(actualTipWorldM - targetTipWorldM);
statusLabel = sprintf("replay=%d/%d_%s_targetError=%.4f", idx, numSamples, seg.strokePlan.plan.status, toolTipErrorM);

poseCommand = struct( ...
    'qCandidate', qCandidate, ...
    'solved', solved, ...
    'toolTipWorldM', actualTipWorldM, ...
    'statusLabel', "AUTO:" + statusLabel, ...
    'toolTipErrorM', toolTipErrorM, ...
    'targetTipWorldM', targetTipWorldM, ...
    'desiredTipWorldM', targetTipWorldM, ...
    'safeTipWorldM', targetTipWorldM, ...
    'governorActive', false, ...
    'cutFlag', cutFlag);

executionState.trajectoryIndex = idx + 1;
completedSegment = executionState.trajectoryIndex > numSamples;
if ~solved
    executionState.trajectoryIndex = 0;
    executionState.isActive = false;
    return;
end
if ~completedSegment
    return;
end

info.segmentJustCompleted = true;
if seg.kind == "stroke"
    executionState.strokesDone = executionState.strokesDone + 1;
    info.strokesDone = executionState.strokesDone;
end
if executionState.queuePos < numel(executionState.queue)
    executionState.queuePos = executionState.queuePos + 1;
    [executionState, info.nextSegmentInfo] = loadSegmentAt(executionState, executionState.queuePos);
else
    executionState.trajectoryIndex = 0;
    executionState.isActive = false;
    info.runComplete = true;
end
end
