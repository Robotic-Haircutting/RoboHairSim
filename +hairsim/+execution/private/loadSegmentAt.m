function [executionState, segmentInfo] = loadSegmentAt(executionState, pos)
%LOADSEGMENTAT Load queue position pos's data into currentSegment and
%   reset trajectoryIndex to 1. Ported from run_robot_haircut.m's
%   startQueuedStroke (minus updateSessionConfigFromAutoPanel/
%   setPlanStatusText, which are App-shell/session concerns).
regionPlan = executionState.regionPlan;
segment = executionState.queue(pos);
numStrokesInRun = nnz([executionState.queue.kind] == "stroke");

pendingStationSwitch = struct('switchIdx', 0, 'toThetaRad', nan);
if segment.kind == "stroke"
    stroke = regionPlan.strokes(segment.index);
    savedPlanResult = stroke.planResult;
    planSummary = savedPlanResult.summary;
    planSummary.regionId = regionPlan.region;
    segmentInfo = struct('kind', "stroke", 'laneIdx', stroke.laneIdx, ...
        'strokeOrdinal', executionState.strokesDone + 1, 'numStrokesInRun', numStrokesInRun, ...
        'planSummary', planSummary);
else
    transit = regionPlan.transits(segment.index);
    savedPlanResult = transit.planResult;
    segmentInfo = struct('kind', "transit", ...
        'fromLaneIdx', regionPlan.strokes(transit.fromStroke).laneIdx, ...
        'toLaneIdx', regionPlan.strokes(transit.toStroke).laneIdx);
    if isfield(transit, 'usedRailSlide') && transit.usedRailSlide
        pendingStationSwitch = struct('switchIdx', transit.stationSwitchSampleIdx, ...
            'toThetaRad', transit.railSlideThetaRad);
    end
end

executionState.currentSegment = struct( ...
    'kind', segment.kind, ...
    'strokePlan', savedPlanResult.strokePlan, ...
    'replayPositions', savedPlanResult.replayPositions, ...
    'replayCutFlags', savedPlanResult.cutMask, ...
    'pendingStationSwitch', pendingStationSwitch);
executionState.trajectoryIndex = 1;
segmentInfo.numSamples = size(savedPlanResult.replayPositions, 2);
segmentInfo.numCutSamples = nnz(savedPlanResult.cutMask);
end
