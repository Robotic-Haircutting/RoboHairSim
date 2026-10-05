function result = replayBundle(varargin)
%REPLAYBUNDLE Replay a saved full-demo bundle without planning (returns the result).
%   Puts this version on the MATLAB path (setupPath), then calls
%   hairsim.app.replayFullDemoBundle with the same arguments -- see its help.
setupPath();
result = hairsim.app.replayFullDemoBundle(varargin{:});
end
