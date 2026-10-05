function names = listRegionPlanners()
%LISTREGIONPLANNERS Names of all registered region planners (built-in
%   presets plus anything added with hairsim.planning.registerRegionPlanner),
%   sorted. GUIs build their planner list from this, so a build that ships
%   fewer planners never offers one it cannot run.

ensureBuiltinRegionPlannersRegistered();
names = sort(regionPlannerStore("names"));
end
