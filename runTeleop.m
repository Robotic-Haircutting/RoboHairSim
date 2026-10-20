function runTeleop(varargin)
%RUNTELEOP Drive the robot's clipper over the scalp with the keyboard.
%   Click the 3D view first. W / Up pushes along the feed and is the only
%   move that cuts, with Space held; G turns the feed against the hair
%   flow under the clipper. The panel lists all keys. Every command goes
%   through the same pose construction, safety checks and cut rule as a
%   planned pass, so a hand-guided pass cuts exactly what the same planned
%   pass would; a command that would hit the head is refused and the
%   reason shown. Recording is optional.
%
%   runTeleop()
%   runTeleop('RegionName', "scalp_left", 'TargetLengthM', 0.01)
%
%   Options (name-value, all optional):
%     AdapterName    robot: "ur5e" (default) or "gen3"
%     RegionName     confine the clipper and the cut to one region,
%                    default "" (whole head)
%     TargetLengthM  guard-comb length left on the head, default 0.005
%     PlannerName    whose cut rule to use, default "camClipperAgainstFlow"
%     DataDir        folder with your own head and hair data
%   See also runAuto, hairsim.app.runTeleopSession.
setupPath();
hairsim.app.runTeleopSession(varargin{:});
end
