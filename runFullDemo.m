function result = runFullDemo(varargin)
%RUNFULLDEMO Plan and cut all five scalp regions in one session.
%   The regions are planned and executed in turn on one shared hair state
%   (hair cut in one region stays cut when the next is planned), joined
%   by collision-checked transits and, on the rail, rail slides. Returns
%   per-region coverage and the recording; the whole joint trajectory can
%   be saved as a bundle and replayed later without planning.
%
%   result = runFullDemo()
%   result = runFullDemo('BasePlacementId', "rail180", 'TargetLengthM', 0.005, ...
%       'SaveBundlePath', "outputs/fullhead_bundle.mat")
%
%   Options (name-value, all optional):
%     BasePlacementId  "fixed" (default), "ceiling", "rail<deg>" (start on
%                      that rail station, e.g. "rail180"; the planner still
%                      picks a station per stroke) or "railPerRegion"
%     TargetLengthM    guard-comb length left on the head, default 0.005
%     RegionNames      regions in execution order (default: all five)
%     SaveBundlePath   .mat file for replayBundle, default "" (not saved)
%     ShowScene        false = no window (faster), default true
%     AdapterName      "ur5e" (default) or "gen3"
%   See also replayBundle, hairsim.app.runFullDemoSession (all options and
%   the result fields).
setupPath();
result = hairsim.app.runFullDemoSession(varargin{:});
end
