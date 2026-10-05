function result = runFullDemo(varargin)
%RUNFULLDEMO Plan and run all five scalp regions in one session (returns the result).
%   Puts this version on the MATLAB path (setupPath), then calls
%   hairsim.app.runFullDemoSession with the same arguments -- see its help.
setupPath();
result = hairsim.app.runFullDemoSession(varargin{:});
end
