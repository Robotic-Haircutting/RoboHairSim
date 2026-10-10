function registerRegionPlanner(name, plannerFn)
%REGISTERREGIONPLANNER Register a region planner under a name.
%   hairsim.planning.registerRegionPlanner(name, plannerFn)
%
%   plannerFn must have the signature
%     regionPlan = plannerFn(haircutState, regionName, targetLengthM, context, options)
%   -- the contract every built-in planner (see registerBuiltinPresets.m)
%   already follows. To plug in a wholly custom path-planning algorithm,
%   implement this signature directly and register it under a new name.

arguments
    name (1, 1) string
    plannerFn (1, 1) function_handle
end
regionPlannerStore("register", name, plannerFn);
end
