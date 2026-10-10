function registerBuiltinPresets()
%REGISTERBUILTINPRESETS Register the built-in region planners by name.
%   Idempotent (safe to call more than once).
%
%   camClipperAgainstFlow hairsim.planning.camClipperPlanner: clipper tool
%                         model with the feed-direction rule (comb lifts
%                         only hair it is pushed against), CAM toolpaths +
%                         robot post-processing
%
%   The development tree also registers the experiment baselines
%   (hairsim.benchmark.registerBaselinePlanners: geometricRaster); the
%   release build does not ship +benchmark, so it lists only this one.

hairsim.planning.registerRegionPlanner("camClipperAgainstFlow", @(varargin) hairsim.planning.camClipperPlanner( ...
    varargin{:}, 'PlannerName', "camClipperAgainstFlow"));
baselines = "hairsim.benchmark.registerBaselinePlanners";
if ~isempty(which(baselines))
    feval(baselines);
end
end
