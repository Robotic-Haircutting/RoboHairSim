function [hairAppState, flowDirs] = installClipperCut(hairAppState, headMeshWorld, plannerName, targetLengthM, options)
%INSTALLCLIPPERCUT Cut the hair the way a camClipper-family preset does:
%   a uniform target length (the guard-comb length) and that preset's
%   comb-lift cut model (hairsim.hair.cutting.createCombLiftCutModelForPlanner),
%   so a strand is cut exactly when the planner's own coverage predicate
%   says so. Re-install after the hair is rebuilt (length / density change):
%   the goal and the per-strand flow belong to the strands.
%   flowDirs: (numStrands,3) hair-flow direction of every strand.
%   Region: only that region's strands get the target; every other strand's
%   target is its full length, so the clipper never cuts it, even where the
%   blade reaches over the region boundary ("" = the whole head).

arguments
    hairAppState (1, 1) struct
    headMeshWorld (1, 1) struct
    plannerName (1, 1) string
    targetLengthM (1, 1) double {mustBeNonnegative}
    options.Region (1, 1) string = ""
end

haircutState = hairsim.hair.setUniformTargetLengthGoal(hairAppState.hairLayer.haircutState, targetLengthM);
if strlength(options.Region) > 0
    outside = hairsim.hair.getStrandRegions(haircutState)' ~= options.Region;
    goal = haircutState.lengthGoal;
    goal.targetActiveNodeCounts(outside) = size(haircutState.hairState.guideTargets.X, 1);
    goal.targetRemainingLengthM(outside) = goal.initialRemainingLengthM(outside);
    goal.region = options.Region;
    haircutState.lengthGoal = goal;
end
cutModel = hairsim.hair.cutting.createCombLiftCutModelForPlanner(plannerName, haircutState, headMeshWorld);
if isempty(cutModel)
    error("hairsim:NotAClipperPlanner", "'%s' is not a camClipper-family planner.", plannerName);
end
flowDirs = cutModel.flowDirs;
if isempty(flowDirs)          % the preset ignores the feed direction; the flow is still wanted
    flowDirs = hairsim.cam.computeStrandFlowDirs(haircutState, hairsim.tool.createHeadDistanceQuery(headMeshWorld));
end
hairAppState.hairLayer.haircutState = haircutState;
hairAppState.hairLayer.cutting.model = cutModel;
end
