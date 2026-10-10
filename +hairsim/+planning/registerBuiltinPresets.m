function registerBuiltinPresets()
%REGISTERBUILTINPRESETS Register the built-in region planners by name.
%   Idempotent (safe to call more than once).
%
%   camClipper            hairsim.planning.camClipperPlanner: clipper tool
%                         model, CAM toolpaths + robot post-processing
%   camClipperAgainstFlow the same with the feed-direction rule (comb lifts
%                         only hair it is pushed against)
%
%   The development tree also registers the experiment baselines
%   (hairsim.benchmark.registerBaselinePlanners: geometricRaster); the
%   release build does not ship +benchmark, so it lists only these two.

hairsim.planning.registerRegionPlanner("camClipper", @hairsim.planning.camClipperPlanner);
hairsim.planning.registerRegionPlanner("camClipperAgainstFlow", @(varargin) hairsim.planning.camClipperPlanner( ...
    varargin{:}, 'PlannerName', "camClipperAgainstFlow"));
baselines = "hairsim.benchmark.registerBaselinePlanners";
if ~isempty(which(baselines))
    feval(baselines);
end
end
