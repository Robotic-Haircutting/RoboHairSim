function T_world_tcp = buildTcpPose(footWorldM, normalOutWorld, feedWorld, tool, options)
%BUILDTCPPOSE Clipper TCP pose over one scalp point.
%   z_tcp = -n (toward the scalp), x_tcp = feed direction projected onto
%   the tangent plane, y_tcp = z x x. The TCP sits StandoffM above the foot
%   point along n (default tool.nominalStandoffM).
%   Edge-cut variants (CAM corner cleanup):
%     LateralOffsetM  shifts the TCP along y_tcp, so the foot point lies
%                     under one side of the blade instead of its centre;
%     RollRad         rolls the clipper about x_tcp (feed axis) through
%                     the foot point's normal line, so one edge of the
%                     blade drops toward the scalp.

arguments
    footWorldM (1, 3) double
    normalOutWorld (1, 3) double
    feedWorld (1, 3) double
    tool (1, 1) struct
    options.StandoffM (1, 1) double = tool.nominalStandoffM
    options.LateralOffsetM (1, 1) double = 0
    options.RollRad (1, 1) double = 0
end

n = normalOutWorld / norm(normalOutWorld);
x = feedWorld - dot(feedWorld, n) * n;
if norm(x) < 1e-9
    error("hairsim:FeedParallelToNormal", "Feed direction is parallel to the scalp normal.");
end
x = x / norm(x);
z = -n;
y = cross(z, x);
R = [x(:), y(:), z(:)];
if options.RollRad ~= 0
    c = cos(options.RollRad);
    s = sin(options.RollRad);
    R = R * [1, 0, 0; 0, c, -s; 0, s, c];
end
T_world_tcp = eye(4);
T_world_tcp(1:3, 1:3) = R;
T_world_tcp(1:3, 4) = (footWorldM + options.StandoffM * n + options.LateralOffsetM * y)';
end
