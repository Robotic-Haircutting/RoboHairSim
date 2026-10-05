function runAuto(varargin)
%RUNAUTO Interactive Auto: plan a scalp region, preview, run, record.
%   Puts this version on the MATLAB path (setupPath), then calls
%   hairsim.app.runAutoSession with the same arguments -- see its help.
setupPath();
hairsim.app.runAutoSession(varargin{:});
end
