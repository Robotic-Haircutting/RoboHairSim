function [executionState, firstStroke] = loadPlan(regionPlan)
%LOADPLAN Build the run queue (stroke, transit, stroke, ... over the
%   plan's chain of continuously reachable strokes) from a finished
%   regionPlan. Ported from run_robot_haircut.m's buildRunSegments.
%   Returns a freshly-idle executionState (queuePos=0 -- call
%   hairsim.execution.startRun to actually begin) plus firstStroke, which
%   the caller must SETUP-jump the robot to (place it on firstStroke's
%   first sample -- there is no planned transit INTO the first stroke of a
%   chain, exactly as the original's applySetupJumpToStroke only ever ran
%   once, before the run, never between segments) before calling startRun.
%
%   Errors if regionPlan is not executable (mirrors the original's
%   "Auto has no other planner: without a current plan this refuses").

arguments
    regionPlan (1, 1) struct
end
if ~regionPlan.executable
    error("hairsim:RegionPlanNotExecutable", ...
        "The plan has no executable stroke; it cannot be loaded for execution.");
end

queue = struct('kind', {}, 'index', {});
chain = regionPlan.chainStrokeIdx;
for chainPos = 1:numel(chain)
    if chainPos > 1
        transitIdx = find([regionPlan.transits.fromStroke] == chain(chainPos - 1) & ...
            [regionPlan.transits.toStroke] == chain(chainPos) & ...
            [regionPlan.transits.feasible], 1);
        queue(end + 1) = struct('kind', "transit", 'index', transitIdx); %#ok<AGROW>
    end
    queue(end + 1) = struct('kind', "stroke", 'index', chain(chainPos)); %#ok<AGROW>
end

executionState = struct( ...
    'regionPlan', regionPlan, ...
    'queue', queue, ...
    'queuePos', 0, ...
    'strokesDone', 0, ...
    'currentSegment', [], ...
    'trajectoryIndex', 0, ...
    'isActive', false);

firstStroke = regionPlan.strokes(queue(1).index);
end
