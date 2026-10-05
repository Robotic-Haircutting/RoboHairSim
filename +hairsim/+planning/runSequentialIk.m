function [qTrajectory, info] = runSequentialIk(primitive, ctx, params, ik, progressFn, laneLabel, qStart, candidateGenFn)
%RUNSEQUENTIALIK Walk every sample of primitive (APPROACH -> CUT -> RETREAT)
%   in order, each solved with the previous sample's joint solution as the
%   first seed (fast attempt, then an extended attempt with +-90 deg
%   seeds). Any sample without an accepted candidate (see
%   hairsim.planning.solveAndPick) fails the WHOLE walk immediately -- this
%   is the shared stepping logic behind the "failLane" lane-recovery
%   strategy, and is also reused (with a 1-sample primitive and no fallback
%   seeds needed) for inter-stroke/transit segments by the transit
%   strategies, exactly as the original planAutonomousRegionHaircut.m used
%   one runSequentialIk for both strokes and transits.
%
%   qStart (optional): the configuration a transit continues from; a
%   stroke starts from ctx.qHome instead (qStart omitted or empty). Every
%   sample including the first is then held to the joint-step limit.
%
%   candidateGenFn: the active orientation-strategy tier (see
%   hairsim.planning.orientationStrategies.strict), passed through to
%   every hairsim.planning.solveAndPick call.

numSamples = size(primitive.pointsWorldM, 1);
qTrajectory = nan(ctx.numJoints, numSamples);
info = struct('ok', false, 'reason', "", 'failedSample', 0, 'detail', "");

hasStart = nargin >= 7 && ~isempty(qStart);
qPrev = ctx.qHome;
if hasStart
    qPrev = qStart(:);
end
for k = 1:numSamples
    reportProgress(progressFn, sprintf("%s: IK %d/%d", laneLabel, k, numSamples));
    enforceStep = k > 1 || hasStart;
    if k == 1 && ~hasStart
        [q, rejected] = hairsim.planning.solveAndPick(ctx, params, ik, qPrev, primitive, k, "first", enforceStep, candidateGenFn);
    else
        [q, rejected] = hairsim.planning.solveAndPick(ctx, params, ik, qPrev, primitive, k, "fast", enforceStep, candidateGenFn);
        if isempty(q)
            [q, rejected] = hairsim.planning.solveAndPick(ctx, params, ik, qPrev, primitive, k, "extended", enforceStep, candidateGenFn);
        end
    end
    if isempty(q)
        info.reason = "no_ik_candidate";
        info.failedSample = k;
        info.detail = sprintf(['%s sample %d/%d: rejected notSolved=%d robotHead=%d ', ...
            'jointStep=%d clipperClearance=%d selfCollision=%d'], ...
            primitive.samplePhase(k), k, numSamples, rejected.notSolved, rejected.robotHead, ...
            rejected.jointStep, rejected.clipperClearance, rejected.selfCollision);
        return;
    end
    qTrajectory(:, k) = q;
    qPrev = q;
end
info.ok = true;
end

function reportProgress(progressFn, message)
if ~isempty(progressFn)
    progressFn(char(message));
end
end
