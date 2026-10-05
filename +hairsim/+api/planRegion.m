function regionPlan = planRegion(haircutState, regionName, targetLengthM, context, plannerName, options)
%PLANREGION Plan one scalp region with a named planner, as a library call.
%   regionPlan = hairsim.api.planRegion(haircutState, regionName,
%   targetLengthM, context, "strict")
%
%   plannerName is looked up via hairsim.planning.getRegionPlanner --
%   "strict", "adaptive", "intralane" are built in (see
%   hairsim.planning.registerBuiltinPresets); register your own with
%   hairsim.planning.registerRegionPlanner and pass its name here.
%
%   options passes through to the planner (OnProgress, LaneIndices,
%   PlanTransits -- see hairsim.planning.planRegionCore).

arguments
    haircutState (1, 1) struct
    regionName (1, 1) string
    targetLengthM (1, 1) double {mustBeFinite, mustBeNonnegative}
    context (1, 1) struct
    plannerName (1, 1) string
    options.OnProgress = []
    options.LaneIndices (1, :) double = []
    options.PlanTransits (1, 1) logical = false
end

plannerFn = hairsim.planning.getRegionPlanner(plannerName);
regionPlan = plannerFn(haircutState, regionName, targetLengthM, context, ...
    'OnProgress', options.OnProgress, 'LaneIndices', options.LaneIndices, ...
    'PlanTransits', options.PlanTransits);
end
