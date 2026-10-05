function [transit, replannedStroke] = direct(strokeA, strokeB, regionGeometry, ctx, params, progressFn, label)
%DIRECT Transit strategy: LIFT -> TRANSFER -> LOWER from the end of
%   strokeA to the start of strokeB, base fixed throughout. This is
%   planAutonomousRegionHaircut.m's original (and still default) inter-
%   stroke transit, unchanged by any lane-recovery or orientation strategy
%   -- it always uses the STRICT (flow-aligned, roll=0) orientation
%   regardless of what the strokes themselves used, exactly as the
%   original 4 planner files all did (transits are LIFT/TRANSFER/LOWER
%   poses above the scalp, not flow-aligned cutting poses, so an
%   orientation fallback's premise doesn't apply here).
%
%   A transit strategy is a function
%     [transit, replannedStroke] = strategyFn(strokeA, strokeB, regionGeometry, ctx, params, progressFn, label)
%   returning hairsim.planning.emptyTransit's shape plus, optionally, a
%   replacement for strokeB (e.g. hairsim.planning.transitStrategies.
%   railSlide replans strokeB from scratch when it slides to a new
%   station) -- [] when strokeB's existing plan is reused as-is, which is
%   always true for direct. regionGeometry (struct: lanes, regionMesh,
%   normalQueryFn, flowWorld -- everything planRegionCore computes once)
%   is ignored here; only a strategy that needs to replan a lane (like
%   railSlide) uses it. Called by the core's greedy chain-building loop
%   (see planRegionCore.m) for every candidate (lastChained, next) pair in
%   frozen order -- that loop itself is shared and unchanged by which
%   transit strategy is active (confirmed against
%   planAutonomousRegionHaircutRailSlideTransit.m's own docs: it replaces
%   only this function, not the chain-building loop around it).
%
%   Same machinery and gates as a stroke (sequential IK seeded from the
%   configuration strokeA actually ends in; robot/clipper clear of the
%   head, sphere clearance, arm self-collision, joint step <= 60 deg on
%   every sample), no cutting (cutMask is all false). The tool orientation
%   stays at strokeA's end orientation while lifting, is interpolated
%   (rotation about the relative axis) during the transfer, and equals
%   strokeB's start orientation on the way down. strokeB's own joint path
%   was planned independently, so the last transit pose is joined to its
%   first configuration by a short joint-space blend (<= 5 deg per sample)
%   that is validated like every other sample. Ported unchanged from
%   planAutonomousRegionHaircut.m's own planTransit.

strictCandidateGenFn = strictTier();
replannedStroke = [];

transit = hairsim.planning.emptyTransit();
timer = tic;
primA = strokeA.primitive;
primB = strokeB.primitive;
lastCutA = find(primA.samplePhase == "CUT", 1, 'last');
firstCutB = find(primB.samplePhase == "CUT", 1, 'first');
pA = primA.pointsWorldM(end, :);
pB = primB.pointsWorldM(1, :);
nA = hairsim.planning.unitVector(pA - primA.pointsWorldM(lastCutA, :));
nB = hairsim.planning.unitVector(pB - primB.pointsWorldM(firstCutB, :));
pLiftA = strokeA.scalpCutPathWorldM(end, :) + params.transferHeightM * nA;
pLiftB = strokeB.scalpCutPathWorldM(1, :) + params.transferHeightM * nB;
RA = primA.R_world_ee(:, :, end);
RB = primB.R_world_ee(:, :, 1);
qStart = strokeA.planResult.replayPositions(:, end);
qTarget = strokeB.planResult.replayPositions(:, 1);

relative = rotm2axang(RA' * RB);
angleAB = relative(4);
numLift = max(1, ceil(norm(pLiftA - pA) / params.transitStepM));
numTransfer = max([1, ceil(norm(pLiftB - pLiftA) / params.transitStepM), ceil(angleAB / params.transitStepRad)]);
numLower = max(1, ceil(norm(pB - pLiftB) / params.transitStepM));
liftT = (1:numLift)' / numLift;
transferT = (1:numTransfer)' / numTransfer;
lowerT = (1:numLower)' / numLower;
points = [pA + liftT .* (pLiftA - pA); pLiftA + transferT .* (pLiftB - pLiftA); pLiftB + lowerT .* (pB - pLiftB)];
numSamples = size(points, 1);
rotations = zeros(3, 3, numSamples);
for k = 1:numLift
    rotations(:, :, k) = RA;
end
for k = 1:numTransfer
    rotations(:, :, numLift + k) = RA * axang2rotm([relative(1:3), transferT(k) * angleAB]);
end
for k = 1:numLower
    rotations(:, :, numLift + numTransfer + k) = RB;
end
phase = [repmat("LIFT", numLift, 1); repmat("TRANSFER", numTransfer, 1); repmat("LOWER", numLower, 1)];
primitive = struct('pointsWorldM', points, 'R_world_ee', rotations, 'samplePhase', phase);

ik = inverseKinematics('RigidBodyTree', ctx.robot);
ik.SolverParameters.AllowRandomRestart = false;
[qIk, ikInfo] = hairsim.planning.runSequentialIk(primitive, ctx, params, ik, progressFn, label, qStart, strictCandidateGenFn);
if ~ikInfo.ok
    transit.reason = ikInfo.reason;
    transit.failedSample = ikInfo.failedSample;
    transit.detail = ikInfo.detail;
    transit.planningTimeS = toc(timer);
    return;
end

% join to strokeB's first configuration
qFull = qIk;
gap = qTarget - qIk(:, end);
numBlend = ceil(max(abs(gap)) / params.blendStepRad);
transit.blendMaxJointDeg = rad2deg(max(abs(gap)));
transit.numBlendSamples = numBlend;
blendPoints = zeros(numBlend, 3);
blendRotations = zeros(3, 3, numBlend);
for k = 1:numBlend
    qBlend = qIk(:, end) + gap * k / numBlend;
    qFull(:, end + 1) = qBlend; %#ok<AGROW>
    T = getTransform(ctx.robot, qBlend, char(ctx.endEffector));
    blendPoints(k, :) = hairsim.getMountedClipperPoint(T, ctx.toolMount.T_ee_tool);
    blendRotations(:, :, k) = T(1:3, 1:3);
end
fullPoints = [points; blendPoints];
fullRotations = cat(3, rotations, blendRotations);
fullPhase = [phase; repmat("LOWER", numBlend, 1)];

% validate the whole path including the step out of strokeA's last configuration
checkPrimitive = struct('pointsWorldM', [pA; fullPoints], 'R_world_ee', cat(3, RA, fullRotations));
check = hairsim.planning.validatePlannedPath([qStart, qFull], checkPrimitive, ctx, params);
if ~check.ok
    transit.reason = "transit_check_failed";
    transit.failedSample = max(check.firstBadSample - 1, 1);
    transit.detail = string(check.reason);
    if transit.failedSample > numSamples
        transit.reason = "final_joint_connection_failed";
    end
    transit.planningTimeS = toc(timer);
    return;
end

tips = check.tipsWorldM(2:end, :);
strokePlan = hairsim.motion.createStrokePlan( ...
    check.tipsWorldM(1, :), tips(end, :), getTransform(ctx.robot, qFull(:, end), char(ctx.endEffector)), ...
    qStart, qFull(:, end), qFull, tips, "b_transit_sequential_ik", fullPoints, fullRotations, ...
    fullPhase, "b_transit");
summary = hairsim.motion.summarizeAutoStrokePlan( ...
    strokePlan, ctx.robot, ctx.endEffector, ctx.toolMount, ctx.headMeshSafetyWorld);
planResult = struct();
planResult.strokePlan = strokePlan;
planResult.summary = summary;
planResult.replayPositions = qFull;
planResult.cutMask = false(1, size(qFull, 2));
planResult.worldObstacleCount = 1;
planResult.replayable = summary.success;
planResult.message = "OK";
if ~summary.success
    planResult.message = "summary.success is false: " + string(summary.failureReason);
end
transit.feasible = true;
transit.samplePhase = fullPhase;
transit.achievedPathWorldM = check.centersWorldM(2:end, :);
transit.check = rmfield(check, {'centersWorldM', 'tipsWorldM'});
transit.planResult = planResult;
transit.planningTimeS = toc(timer);
end

function candidateGenFn = strictTier()
candidateGenFn = @(R0, ~) struct('baseRotation', R0, 'rollAnglesRad', 0);
end
