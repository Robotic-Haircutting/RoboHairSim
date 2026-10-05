function input = emptyInput()
%EMPTYINPUT No queued Teleoperation input (hairsim.app.teleop.handleKey
%   accumulates into this; hairsim.app.teleop.stepTeleop consumes it).
%   Counts are key presses: feed (+forward / -back), side (+right / -left,
%   seen from outside facing the feed), yaw (+left turn about the normal),
%   standoff (+lift / -lower), roll (+/-); snapToFlow: turn the feed
%   against the hair flow under the clipper.
input = struct('feed', 0, 'side', 0, 'yaw', 0, 'standoff', 0, 'roll', 0, 'snapToFlow', false);
end
