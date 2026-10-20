function [executionState, firstStroke] = loadPlan(regionPlan)
%LOADPLAN Build the run queue (stroke, transit, stroke, ... over the
%   plan's chain of continuously reachable strokes) from a finished
%   regionPlan.
%   Returns a freshly-idle executionState (queuePos=0 -- call
%   hairsim.execution.startRun to actually begin) plus firstStroke, which
%   the caller must SETUP-jump the robot to (place it on firstStroke's
%   first sample -- there is no planned transit INTO the first stroke of a
%   chain: the setup jump happens once, before the run, never between
%   segments) before calling startRun.
%
%   Errors if regionPlan is not executable or its chain is empty (strokes
%   executable, but none connected to the park pose).

arguments
    regionPlan (1, 1) struct
end
if ~regionPlan.executable
    error("hairsim:RegionPlanNotExecutable", ...
        "The plan has no executable stroke; it cannot be loaded for execution.");
end

chain = regionPlan.chainStrokeIdx;
if isempty(chain)
    error("hairsim:RegionPlanEmptyChain", ...
        "The plan has executable strokes but none in the chain; it cannot be run.");
end
queue = struct('kind', {}, 'index', {});
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
