function [segments, blocked] = buildSegments(passFeet, surface, tool, kind, options)
%BUILDSEGMENTS Turn foot-point passes into clipper segments (CL data).
%   Each pass is split into pieces of SegmentLengthM with OverlapM of
%   overlap (the user-approved "split long lanes"), every foot point gets
%   a TCP pose (hairsim.tool.buildTcpPose: z = -smoothed normal, x = local
%   feed tangent, optional edge-cut offset/roll), and every pose is lifted
%   along the normal only as far as the comb tolerance allows
%   (hairsim.tool.resolveContactStandoff). Poses that cannot clear the head
%   within the tolerance are BLOCKED: the piece is split around them and
%   they are returned for rest/edge machining. Each surviving run gets a
%   straight APPROACH and RETREAT along its end normals.
%
%   passFeet: cell of (K,3) feet in feed order. kind: "main" | "rest" | "edge".
%   segments: struct array, see emptySegment below. blocked: (B,3) feet.

arguments
    passFeet (1, :) cell
    surface (1, 1) struct
    tool (1, 1) struct
    kind (1, 1) string
    options.SegmentLengthM (1, 1) double {mustBePositive} = 0.04
    options.OverlapM (1, 1) double {mustBeNonnegative} = 0.01
    options.ApproachM (1, 1) double {mustBeNonnegative} = 0.02
    options.ApproachStepM (1, 1) double {mustBePositive} = 0.005
    options.LateralOffsetM (1, 1) double = 0
    options.RollRad (1, 1) double = 0
    options.MinRunPoses (1, 1) double {mustBePositive, mustBeInteger} = 2
end

segments = repmat(emptySegment(), 1, 0);
blocked = zeros(0, 3);
for p = 1:numel(passFeet)
    feet = passFeet{p};
    for piece = splitByLength(feet, options.SegmentLengthM, options.OverlapM)
        f = piece{1};
        normals = hairsim.cam.smoothedNormalAt(f, surface.headQuery);
        tangents = localTangents(f);
        numPoses = size(f, 1);
        T = zeros(4, 4, numPoses);
        ok = false(numPoses, 1);
        lift = nan(numPoses, 1);
        baseStandoff = tool.nominalStandoffM + tool.bladeHalfWidthM * abs(sin(options.RollRad));
        for k = 1:numPoses
            Tn = hairsim.tool.buildTcpPose(f(k, :), normals(k, :), tangents(k, :), tool, ...
                'StandoffM', baseStandoff, 'LateralOffsetM', options.LateralOffsetM, ...
                'RollRad', options.RollRad);
            [T(:, :, k), ok(k), lift(k)] = hairsim.tool.resolveContactStandoff( ...
                Tn, normals(k, :), tool, surface.headQuery);
        end
        blocked = [blocked; f(~ok, :)]; %#ok<AGROW>
        edges = diff([false; ok; false]);
        starts = find(edges == 1);
        stops = find(edges == -1) - 1;
        for r = 1:numel(starts)
            idx = starts(r):stops(r);
            if numel(idx) < options.MinRunPoses
                blocked = [blocked; f(idx, :)]; %#ok<AGROW>
                continue;
            end
            seg = emptySegment();
            seg.region = surface.region;
            seg.kind = kind;
            seg.passIdx = p;
            seg.footWorldM = f(idx, :);
            seg.normalWorldM = normals(idx, :);
            seg.liftM = lift(idx);
            seg.lateralOffsetM = options.LateralOffsetM;
            seg.rollRad = options.RollRad;
            seg.T_cut = T(:, :, idx);
            [seg.T_world_tcp, seg.samplePhase] = hairsim.cam.withApproachRetreat(seg.T_cut, ...
                normals(idx(1), :), normals(idx(end), :), options.ApproachM, options.ApproachStepM);
            segments(end + 1) = seg; %#ok<AGROW>
        end
    end
end
end

function seg = emptySegment()
seg = struct('region', "", 'kind', "", 'passIdx', 0, 'footWorldM', zeros(0, 3), ...
    'normalWorldM', zeros(0, 3), 'liftM', zeros(0, 1), 'lateralOffsetM', 0, 'rollRad', 0, ...
    'T_cut', zeros(4, 4, 0), 'T_world_tcp', zeros(4, 4, 0), 'samplePhase', strings(0, 1));
end

function pieces = splitByLength(feet, segLen, overlap)
s = [0; cumsum(vecnorm(diff(feet), 2, 2))];
total = s(end);
pieces = {};
if total <= segLen * 1.25
    pieces = {feet};
    return;
end
stride = segLen - overlap;
starts = 0:stride:(total - segLen);       % the last piece runs to the end
for i = 1:numel(starts)
    a = starts(i);
    b = a + segLen;
    if i == numel(starts)
        b = total;
    end
    idx = find(s >= a - 1e-9 & s <= b + 1e-9);
    if numel(idx) >= 2
        pieces{end + 1} = feet(idx, :); %#ok<AGROW>
    end
end
end

function t = localTangents(f)
t = zeros(size(f));
n = size(f, 1);
for k = 1:n
    a = max(1, k - 1);
    b = min(n, k + 1);
    d = f(b, :) - f(a, :);
    t(k, :) = d / max(norm(d), eps);
end
end
