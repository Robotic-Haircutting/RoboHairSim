function runTeleop(varargin)
%RUNTELEOP Interactive Teleoperation: drive the clipper with the keyboard.
%   Puts this version on the MATLAB path (setupPath), then calls
%   hairsim.app.runTeleopSession with the same arguments -- see its help.
setupPath();
hairsim.app.runTeleopSession(varargin{:});
end
