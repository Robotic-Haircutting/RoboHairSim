function plannerFn = getRegionPlanner(name)
%GETREGIONPLANNER Look up a registered region planner by name.
%   Lazily registers the built-in planners (camClipperAgainstFlow;
%   geometricRaster in the development tree) on first use.

arguments
    name (1, 1) string
end
ensureBuiltinRegionPlannersRegistered();
plannerFn = regionPlannerStore("get", name);
end
