function flowDirs = computeStrandFlowDirs(haircutState, headQuery, options)
%COMPUTESTRANDFLOWDIRS Hair-flow direction of EVERY strand, by the same
%   rule as hairsim.cam.buildMachiningSurface (rest-shape guide node at
%   mid-length minus root, projected onto the tangent plane of the smoothed
%   scalp normal). Needed by a direction-checking comb-lift cut model,
%   which may cut any strand (the toolpaths cross region borders).
%
%   flowDirs: (numStrands, 3) unit vectors.

arguments
    haircutState (1, 1) struct
    headQuery (1, 1) struct
    options.NormalRadiusM (1, 1) double {mustBePositive} = 0.006
end

hairs = haircutState.hairState.hairs;
guides = haircutState.hairState.guideTargets;
roots = [hairs.X(1, :)', hairs.Y(1, :)', hairs.Z(1, :)'];
normals = hairsim.cam.smoothedNormalAt(roots, headQuery, options.NormalRadiusM);
mid = ceil(size(guides.X, 1) / 2);
growth = [guides.X(mid, :)', guides.Y(mid, :)', guides.Z(mid, :)'] - roots;
flowDirs = growth - sum(growth .* normals, 2) .* normals;
flowDirs = flowDirs ./ max(vecnorm(flowDirs, 2, 2), eps);
end
