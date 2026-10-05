function cutModel = createCombLiftCutModelForPlanner(plannerName, haircutState, headMeshWorld)
%CREATECOMBLIFTCUTMODELFORPLANNER The comb-lift cut model a camClipper-family
%   plan must be executed / replayed with: the preset's own tool
%   (hairsim.planning.camClipperTool) and, when that tool checks the feed
%   direction, every strand's hair flow (hairsim.cam.computeStrandFlowDirs).
%   Returns [] for a planner outside the camClipper family.

arguments
    plannerName (1, 1) string
    haircutState (1, 1) struct
    headMeshWorld (1, 1) struct
end

[tool, isCamClipper] = hairsim.planning.camClipperTool(plannerName);
cutModel = [];
if ~isCamClipper
    return;
end
flowDirs = zeros(0, 3);
if isfinite(tool.maxFeedAngleRad)
    flowDirs = hairsim.cam.computeStrandFlowDirs(haircutState, ...
        hairsim.tool.createHeadDistanceQuery(headMeshWorld));
end
cutModel = hairsim.hair.cutting.createCombLiftCutModel(tool, 'FlowDirs', flowDirs);
end
