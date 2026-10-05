function check = validatePlannedPath(qTrajectory, primitive, ctx, params)
%VALIDATEPLANNEDPATH Recompute every safety/accuracy quantity from the
%   finished joint path alone (forward kinematics), independent of the
%   per-candidate gates that produced it. Shared by every lane-recovery and
%   transit strategy (ported unchanged from
%   planAutonomousRegionHaircut.m's own validatePlannedPath).
numSamples = size(qTrajectory, 2);
centers = zeros(numSamples, 3);
tips = zeros(numSamples, 3);
orientationErrorRad = zeros(numSamples, 1);
robotHeadClear = false(numSamples, 1);
selfCollision = false(numSamples, 1);
for k = 1:numSamples
    T = getTransform(ctx.robot, qTrajectory(:, k), char(ctx.endEffector));
    centers(k, :) = hairsim.getMountedClipperPoint(T, ctx.toolMount.T_ee_tool_center);
    tips(k, :) = hairsim.getMountedClipperPoint(T, ctx.toolMount.T_ee_tool);
    cosAngle = (trace(primitive.R_world_ee(:, :, k)' * T(1:3, 1:3)) - 1) / 2;
    orientationErrorRad(k) = acos(min(1, max(-1, cosAngle)));
    robotHeadClear(k) = hairsim.motion.validateClipperRobotCollision( ...
        ctx.robot, qTrajectory(:, k), ctx.endEffector, ctx.toolMount, ...
        ctx.headCollision, params.collisionMarginM, true, 'SkipClipperHeadCheck', false);
    selfCollision(k) = checkCollision(ctx.armOnlyRobot, qTrajectory(:, k), ...
        'SkippedSelfCollisions', 'parent');
end
tipErrorM = vecnorm(tips - primitive.pointsWorldM, 2, 2);
clearance = hairsim.evaluateClipperSphereMeshClearance( ...
    centers, ctx.headMeshSafetyWorld, ctx.toolMount.toolClearanceRadiusM);
jointStepRad = max(abs(diff(qTrajectory, 1, 2)), [], 1)';

check = struct();
check.centersWorldM = centers;
check.tipsWorldM = tips;
check.maxTipErrorM = max(tipErrorM);
check.minSurfaceClearanceM = clearance.minSurfaceClearanceM;
check.headPenetrationSampleCount = clearance.headPenetrationSampleCount;
check.robotHeadCollisionSampleCount = nnz(~robotHeadClear);
check.selfCollisionSampleCount = nnz(selfCollision);
check.maxJointStepDeg = rad2deg(max([jointStepRad; 0]));
check.orientationErrorDeg = rad2deg(orientationErrorRad);
check.numSamples = numSamples;

badReasons = [ ...
    "tip error above tolerance", ...
    "clipper sphere within 1 mm of / inside the head", ...
    "robot or clipper collides with the head", ...
    "self collision", ...
    "joint step above 60 deg", ...
    "non-finite joint values"];
badMasks = [ ...
    tipErrorM > params.maxToolTipErrorM, ...
    clearance.surfaceClearanceM < params.dSafeM, ...
    ~robotHeadClear, ...
    selfCollision, ...
    [false; jointStepRad > params.maxJointStepRad + 1e-12], ...
    ~all(isfinite(qTrajectory), 1)'];

firstBadSample = 0;
firstBadReason = "OK";
for maskIdx = 1:size(badMasks, 2)
    badIdx = find(badMasks(:, maskIdx), 1, 'first');
    if ~isempty(badIdx) && (firstBadSample == 0 || badIdx < firstBadSample)
        firstBadSample = badIdx;
        firstBadReason = badReasons(maskIdx);
    end
end
check.ok = firstBadSample == 0;
check.firstBadSample = firstBadSample;
check.reason = firstBadReason;
if ~check.ok
    check.reason = sprintf("%s (sample %d)", firstBadReason, firstBadSample);
end
end
