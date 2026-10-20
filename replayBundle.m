function result = replayBundle(varargin)
%REPLAYBUNDLE Replay a saved full-head session without planning.
%   Plays back the joint trajectory stored by runFullDemo(...,
%   'SaveBundlePath', file): no planning or IK, so it is fast and gives
%   exactly the same cut. The hair is simulated and cut again as it plays.
%   Optionally writes a video.
%
%   result = replayBundle("outputs/fullhead_bundle.mat")
%   result = replayBundle("outputs/fullhead_bundle.mat", 'VideoPath', "fullhead.mp4", ...
%       'CameraMode', "follow")
%
%   Options (name-value, all optional):
%     VideoPath         MPEG-4 file to write, default "" (no video)
%     VideoFps          frames per second, default 30
%     VideoFrameStride  write every n-th tick, default 2
%     CameraMode        "fixed" (default) or "follow" (follows the clipper)
%     ShowScene         false = no window (no video either), default true
%   See also runFullDemo, hairsim.app.replayFullDemoBundle (all options).
setupPath();
result = hairsim.app.replayFullDemoBundle(varargin{:});
end
