function teleopState = handleKey(teleopState, keyName, isPressed)
%HANDLEKEY Record one key press/release into the queued input. Pure state
%   update -- nothing moves until hairsim.app.teleop.stepTeleop.
%     W / Up      push forward along the feed (the only move that cuts)
%     S / Down    pull back
%     A D / Left Right   slide sideways (along the blade width)
%     Q / E       turn the feed left / right about the scalp normal
%     G           turn the feed against the hair flow under the clipper
%     R / F       lift / lower the clipper (lowest = touching the scalp)
%     Z / X       roll the blade (edge cut), within +-MaxRollRad
%     C / V       slide the base along the rail (rail setups only)
%     Space       hold to cut

arguments
    teleopState (1, 1) struct
    keyName
    isPressed (1, 1) logical
end

key = lower(char(keyName));
if strcmp(key, 'space')
    teleopState.spaceHeld = isPressed;
    return;
end
if ~isPressed
    return;
end
p = teleopState.pending;
switch key
    case {'w', 'uparrow'}
        p.feed = p.feed + 1;
    case {'s', 'downarrow'}
        p.feed = p.feed - 1;
    case {'d', 'rightarrow'}
        p.side = p.side + 1;
    case {'a', 'leftarrow'}
        p.side = p.side - 1;
    case 'q'
        p.yaw = p.yaw + 1;
    case 'e'
        p.yaw = p.yaw - 1;
    case 'g'
        p.snapToFlow = true;
    case 'r'
        p.standoff = p.standoff + 1;
    case 'f'
        p.standoff = p.standoff - 1;
    case 'z'
        p.roll = p.roll - 1;
    case 'x'
        p.roll = p.roll + 1;
    case 'c'
        p.rail = p.rail - 1;
    case 'v'
        p.rail = p.rail + 1;
end
teleopState.pending = p;
end
