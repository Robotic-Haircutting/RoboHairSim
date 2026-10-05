function autoSummary = createAutoPlanFailureSummary(startTipWorldM, targetTipWorldM, reason)
%CREATEAUTOPLANFAILURESUMMARY Record failed auto attempts for success-rate tables.

autoSummary = struct();
autoSummary.success = false;
autoSummary.status = "failed";
autoSummary.failureReason = string(reason);
autoSummary.lastExitFlag = nan;
autoSummary.plannerSampleCount = 0;
autoSummary.startTipWorldM = startTipWorldM;
autoSummary.targetTipWorldM = targetTipWorldM;
autoSummary.reachedTipWorldM = [nan, nan, nan];
autoSummary.targetTipErrorM = nan;
autoSummary.targetTipToleranceM = 0.03;
autoSummary.targetEePositionM = [nan, nan, nan];
autoSummary.reachedEePositionM = [nan, nan, nan];
autoSummary.eePositionErrorM = nan;
autoSummary.eeOrientationErrorRad = nan;
autoSummary.targetClipperCenterWorldM = [nan, nan, nan];
autoSummary.reachedClipperCenterWorldM = [nan, nan, nan];
autoSummary.clipperCenterErrorM = nan;
autoSummary.minCenterClearanceM = nan;
autoSummary.minSurfaceClearanceM = nan;
autoSummary.headPenetrationSampleCount = 0;
autoSummary.hasHeadPenetration = false;
autoSummary.qTrajectory = zeros(0, 0);
autoSummary.timestamp = zeros(0, 1);
end
