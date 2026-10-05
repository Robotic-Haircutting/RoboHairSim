function plannerFn = getRegionPlanner(name)
%GETREGIONPLANNER Look up a registered region planner by name.
%   Lazily registers the built-in presets ("strict", "adaptive",
%   "intralane"; "railslide" to follow) on first use.

arguments
    name (1, 1) string
end
ensureBuiltinRegionPlannersRegistered();
plannerFn = regionPlannerStore("get", name);
end
