function [covered, firstHitPose] = sweepCombLiftCoverage(rootsWorldM, T_world_tcp, tool, options)
%SWEEPCOMBLIFTCOVERAGE Which hair roots a clipper pass stands up and cuts.
%   The ONE coverage predicate of the camClipper pipeline: the CAM layer
%   (ideal poses), the post-processor (robot FK poses) and the comb-lift
%   cut model all use it, so planned and executed coverage cannot drift.
%
%   rootsWorldM: (M,3) roots of the strands that still need cutting.
%   T_world_tcp: (4,4,N) consecutive CUT-phase TCP poses of one pass.
%   A root is covered by a pose when, in that pose's TCP frame (x feed,
%   y blade width, z toward the scalp), it lies in the comb footprint
%       x in tool.footprintXRangeM, |y| <= tool.bladeHalfWidthM
%   and the blade above it is close enough for the comb to lift it:
%       -tool.safetyClearanceM <= z <= tool.maxLocalHeightM.
%   Feed direction: when tool.maxFeedAngleRad is finite, the comb only
%   lifts hair it is pushed AGAINST, so additionally
%       angle(TCP x, -flow of the root) <= tool.maxFeedAngleRad
%   with the roots' hair-flow directions given in options.FlowDirs (M,3)
%   (hairsim.cam.buildMachiningSurface flowDirs). Inf (default) = no check.
%   Consecutive poses further apart than options.MaxStepM / MaxStepRad are
%   sub-sampled (translation lerp + rotation slerp) so a fast pass cannot
%   skip roots between ticks.
%
%   covered: (M,1) logical. firstHitPose: (M,1) index (into the ORIGINAL
%   poses, rounded down from a sub-step) of the first covering pose, NaN
%   if never covered.

arguments
    rootsWorldM (:, 3) double
    T_world_tcp (4, 4, :) double
    tool (1, 1) struct
    options.MaxStepM (1, 1) double {mustBePositive} = 0.003
    options.MaxStepRad (1, 1) double {mustBePositive} = deg2rad(5)
    options.FlowDirs (:, 3) double = zeros(0, 3)
end

checkFeed = isfield(tool, 'maxFeedAngleRad') && isfinite(tool.maxFeedAngleRad);
if checkFeed && size(options.FlowDirs, 1) ~= size(rootsWorldM, 1)
    error("hairsim:eval:FeedCheckNeedsFlow", ...
        "tool.maxFeedAngleRad is set: pass FlowDirs, one per root (%d given, %d roots).", ...
        size(options.FlowDirs, 1), size(rootsWorldM, 1));
end

numRoots = size(rootsWorldM, 1);
covered = false(numRoots, 1);
firstHitPose = nan(numRoots, 1);
numPoses = size(T_world_tcp, 3);
if numRoots == 0 || numPoses == 0
    return;
end

% ---- cull roots far from the whole pass
positions = squeeze(T_world_tcp(1:3, 4, :))';
if numPoses == 1
    positions = positions(:)';
end
reach = norm([max(abs(tool.footprintXRangeM)), tool.bladeHalfWidthM, tool.maxLocalHeightM]) + 0.002;
lo = min(positions, [], 1) - reach;
hi = max(positions, [], 1) + reach;
candidate = find(all(rootsWorldM >= lo & rootsWorldM <= hi, 2));
if isempty(candidate)
    return;
end
P = rootsWorldM(candidate, :);
if checkFeed
    against = -options.FlowDirs(candidate, :);
    minCos = cos(tool.maxFeedAngleRad);
end
hit = false(numel(candidate), 1);
hitPose = nan(numel(candidate), 1);

for k = 1:numPoses
    [Rs, ts] = subSteps(T_world_tcp, k, options.MaxStepM, options.MaxStepRad);
    for s = 1:size(ts, 1)
        open = find(~hit);
        if isempty(open)
            break;
        end
        local = (P(open, :) - ts(s, :)) * Rs(:, :, s);
        inside = local(:, 1) >= tool.footprintXRangeM(1) & local(:, 1) <= tool.footprintXRangeM(2) & ...
            abs(local(:, 2)) <= tool.bladeHalfWidthM & ...
            local(:, 3) >= -tool.safetyClearanceM & local(:, 3) <= tool.maxLocalHeightM;
        if checkFeed
            inside = inside & against(open, :) * Rs(:, 1, s) >= minCos;
        end
        newly = open(inside);
        hit(newly) = true;
        hitPose(newly) = k;
    end
end
covered(candidate) = hit;
firstHitPose(candidate) = hitPose;
end

function [Rs, ts] = subSteps(T, k, maxStepM, maxStepRad)
% Pose k, preceded by sub-steps from pose k-1 (exclusive) when the move is large.
R1 = T(1:3, 1:3, k);
t1 = T(1:3, 4, k)';
if k == 1
    Rs = R1;
    ts = t1;
    return;
end
R0 = T(1:3, 1:3, k - 1);
t0 = T(1:3, 4, k - 1)';
angle = acos(min(1, max(-1, (trace(R0' * R1) - 1) / 2)));
numSub = max(1, ceil(max(norm(t1 - t0) / maxStepM, angle / maxStepRad)));
Rs = zeros(3, 3, numSub);
ts = zeros(numSub, 3);
for s = 1:numSub
    a = s / numSub;
    ts(s, :) = (1 - a) * t0 + a * t1;
    Rs(:, :, s) = slerpRotation(R0, R1, a);
end
end

function R = slerpRotation(R0, R1, a)
dR = R0' * R1;
angle = acos(min(1, max(-1, (trace(dR) - 1) / 2)));
if angle < 1e-9
    R = R0;
    return;
end
axis = [dR(3, 2) - dR(2, 3); dR(1, 3) - dR(3, 1); dR(2, 1) - dR(1, 2)] / (2 * sin(angle));
K = [0, -axis(3), axis(2); axis(3), 0, -axis(1); -axis(2), axis(1), 0];
th = a * angle;
R = R0 * (eye(3) + sin(th) * K + (1 - cos(th)) * (K * K));
end
