function runAuto(varargin)
%RUNAUTO Automatic haircut of one scalp region, with the GUI.
%   Choose a robot setup (fixed, ceiling or a rail station), a region and
%   a target length, then press Plan, Preview and Run. Plans are cached
%   in config/plans and reused; a plan made for other settings is shown
%   as Outdated. A run can be recorded.
%
%   runAuto()
%   runAuto('AdapterName', "gen3", 'RegionName', "scalp_back", 'TargetLengthM', 0.01)
%
%   Options (name-value, all optional):
%     AdapterName    robot: "ur5e" (default) or "gen3"
%     RegionName     initial region, default "scalp_top_left"
%     TargetLengthM  guard-comb length left on the head, default 0.005
%     PlannerName    any registered planner, default "camClipperAgainstFlow"
%     DataDir        folder with your own HeadScan.stl, hair_roots.json
%                    and HairClipper.stl (default: the shipped data/)
%   See also runTeleop, runFullDemo, hairsim.app.runAutoSession.
setupPath();
hairsim.app.runAutoSession(varargin{:});
end
