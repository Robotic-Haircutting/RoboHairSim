function [T_world_tcp, ok, liftM, clearance] = resolveContactStandoff(T_nominal, normalWorld, tool, headQuery)
%RESOLVECONTACTSTANDOFF Lift one clipper pose along the scalp normal until
%   the clipper clears the head, within the comb-contact tolerance.
%
%   T_nominal: TCP pose at the nominal stand-off (teeth face
%   tool.nominalStandoffM above the scalp foot point). The pose is moved
%   along normalWorld by the smallest lift in [0, tool.contactToleranceM]
%   that makes hairsim.tool.evaluateClipperClearance ok. ok = false when
%   even the full tolerance does not clear the head -- the comb could not
%   both stay clear and touch the scalp there with this orientation (the
%   CAM layer then tries an edge-cut variant).

arguments
    T_nominal (4, 4) double
    normalWorld (1, 3) double
    tool (1, 1) struct
    headQuery (1, 1) struct
end

n = normalWorld / norm(normalWorld);
    function [T, c] = atLift(lift)
        T = T_nominal;
        T(1:3, 4) = T_nominal(1:3, 4) + lift * n';
        c = hairsim.tool.evaluateClipperClearance(T, tool, headQuery);
    end

[T_world_tcp, clearance] = atLift(0);
liftM = 0;
ok = clearance.ok;
if ok
    return;
end
[T_hi, c_hi] = atLift(tool.contactToleranceM);
if ~c_hi.ok
    T_world_tcp = T_hi;
    clearance = c_hi;
    liftM = tool.contactToleranceM;
    return;
end
lo = 0;
hi = tool.contactToleranceM;
T_world_tcp = T_hi;
clearance = c_hi;
for iter = 1:8
    mid = 0.5 * (lo + hi);
    [T_mid, c_mid] = atLift(mid);
    if c_mid.ok
        hi = mid;
        T_world_tcp = T_mid;
        clearance = c_mid;
    else
        lo = mid;
    end
end
liftM = hi;
ok = true;
end
