function autoSummary = summarizeAutoStrokePlan( ...
        strokePlan, robot, endEffector, toolMount, meshWorld)
%SUMMARIZEAUTOSTROKEPLAN Create quantitative fields for one auto plan.

qTrajectory = strokePlan.plan.positions;
hasTrajectory = ~isempty(qTrajectory);
if hasTrajectory
    T_world_ee_reached = getTransform(robot, qTrajectory(:, end), char(endEffector));
else
    T_world_ee_reached = nan(4, 4);
end

T_world_ee_target = strokePlan.T_world_ee_target;
targetTipWorldM = strokePlan.targetTipWorldM;
if hasTrajectory
    reachedTipWorldM = hairsim.getMountedClipperPoint( ...
        T_world_ee_reached, toolMount.T_ee_tool);
    reachedCenterWorldM = hairsim.getMountedClipperPoint( ...
        T_world_ee_reached, toolMount.T_ee_tool_center);
    reachedEePositionM = T_world_ee_reached(1:3, 4)';
    reachedEuler = rotm2eul(T_world_ee_reached(1:3, 1:3), 'XYZ');
else
    reachedTipWorldM = [nan, nan, nan];
    reachedCenterWorldM = [nan, nan, nan];
    reachedEePositionM = [nan, nan, nan];
    reachedEuler = [nan, nan, nan];
end

targetCenterWorldM = hairsim.getMountedClipperPoint( ...
    T_world_ee_target, toolMount.T_ee_tool_center);
targetEuler = rotm2eul(T_world_ee_target(1:3, 1:3), 'XYZ');
centerSamples = createClipperCenterSamples( ...
    robot, qTrajectory, endEffector, toolMount.T_ee_tool_center);
clearance = hairsim.evaluateClipperSphereMeshClearance( ...
    centerSamples, meshWorld, hairsim.getToolClearanceRadiusM(toolMount));
targetTipErrorM = normOrNan(reachedTipWorldM - targetTipWorldM);
targetTipToleranceM = 0.03;

autoSummary = hairsim.motion.createAutoPlanFailureSummary( ...
    [nan, nan, nan], [nan, nan, nan], "");
autoSummary.success = strokePlan.plan.status ~= "failed" && ...
    strokePlan.plan.status ~= "not_run" && hasTrajectory && ...
    targetTipErrorM <= targetTipToleranceM && ~clearance.hasHeadPenetration;
autoSummary.status = strokePlan.plan.status;
autoSummary.failureReason = "";
if hasTrajectory && targetTipErrorM > targetTipToleranceM
    autoSummary.failureReason = "target_tip_error_exceeded";
elseif clearance.hasHeadPenetration
    autoSummary.failureReason = "head_penetration";
end
autoSummary.lastExitFlag = strokePlan.plan.lastExitFlag;
autoSummary.plannerSampleCount = size(qTrajectory, 2);
autoSummary.startTipWorldM = strokePlan.startTipWorldM;
autoSummary.targetTipWorldM = targetTipWorldM;
autoSummary.reachedTipWorldM = reachedTipWorldM;
autoSummary.targetTipErrorM = targetTipErrorM;
autoSummary.targetTipToleranceM = targetTipToleranceM;
autoSummary.targetEePositionM = T_world_ee_target(1:3, 4)';
autoSummary.reachedEePositionM = reachedEePositionM;
autoSummary.eePositionErrorM = normOrNan(reachedEePositionM - autoSummary.targetEePositionM);
autoSummary.eeOrientationErrorRad = normOrNan(wrapToPi(reachedEuler - targetEuler));
autoSummary.targetClipperCenterWorldM = targetCenterWorldM;
autoSummary.reachedClipperCenterWorldM = reachedCenterWorldM;
autoSummary.clipperCenterErrorM = normOrNan(reachedCenterWorldM - targetCenterWorldM);
autoSummary.minCenterClearanceM = clearance.minCenterClearanceM;
autoSummary.minSurfaceClearanceM = clearance.minSurfaceClearanceM;
autoSummary.headPenetrationSampleCount = clearance.headPenetrationSampleCount;
autoSummary.hasHeadPenetration = clearance.hasHeadPenetration;
autoSummary.qTrajectory = qTrajectory;
autoSummary.timestamp = strokePlan.plan.timestamp;
end

function centersWorldM = createClipperCenterSamples( ...
        robot, qTrajectory, endEffector, T_ee_tool_center)
centersWorldM = nan(size(qTrajectory, 2), 3);
for sampleIdx = 1:size(qTrajectory, 2)
    T_world_ee = getTransform(robot, qTrajectory(:, sampleIdx), char(endEffector));
    centersWorldM(sampleIdx, :) = hairsim.getMountedClipperPoint( ...
        T_world_ee, T_ee_tool_center);
end
end

function value = normOrNan(vector)
if all(isfinite(vector))
    value = norm(vector);
else
    value = nan;
end
end
