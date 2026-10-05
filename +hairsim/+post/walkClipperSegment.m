function walk = walkClipperSegment(T_world_tcp, samplePhase, sctx, params, tool, headQuery)
%WALKCLIPPERSEGMENT Sequential IK over one segment (APPROACH -> CUT ->
%   RETREAT) at one rail station, the clipper counterpart of
%   hairsim.planning.runSequentialIk: sample 1 is solved from Home with
%   Home seeds, every later sample from the previous solution (fast tier,
%   then the extended tier), all held to the joint-step limit. Stops at the
%   first sample with no accepted candidate.
%
%   walk: ok, q (numJoints x N), T_actual (4x4xN FK TCP poses), failedSample,
%   failedPhase, rejected (counts of the failed sample), reason.

numSamples = size(T_world_tcp, 3);
walk = struct('ok', false, 'q', nan(sctx.numJoints, numSamples), ...
    'T_actual', nan(4, 4, numSamples), 'failedSample', 0, 'failedPhase', "", ...
    'rejected', [], 'reason', "", 'thetaRad', sctx.railThetaCurrentRad);
qPrev = sctx.qHome(:);
for k = 1:numSamples
    if k == 1
        [q, T, rej] = hairsim.post.solveClipperPose(sctx, params, qPrev, T_world_tcp(:, :, k), tool, headQuery, "first", false);
    else
        [q, T, rej] = hairsim.post.solveClipperPose(sctx, params, qPrev, T_world_tcp(:, :, k), tool, headQuery, "fast", true);
        if isempty(q)
            [q, T, rej] = hairsim.post.solveClipperPose(sctx, params, qPrev, T_world_tcp(:, :, k), tool, headQuery, "extended", true);
        end
    end
    if isempty(q)
        walk.failedSample = k;
        walk.failedPhase = samplePhase(k);
        walk.rejected = rej;
        walk.reason = dominantReason(rej);
        return;
    end
    walk.q(:, k) = q;
    walk.T_actual(:, :, k) = T;
    qPrev = q;
end
walk.ok = true;
walk.reason = "OK";
end

function reason = dominantReason(rej)
% The gate that rejected the most candidates names the failure (a
% candidate is counted at the FIRST gate it fails, so this is the
% bottleneck constraint); all-zero = the solver produced no candidate.
names = string(fieldnames(rej))';
counts = cellfun(@(f) rej.(f), cellstr(names));
if all(counts == 0)
    reason = "notSolved";
    return;
end
[~, i] = max(counts);
reason = names(i);
end
