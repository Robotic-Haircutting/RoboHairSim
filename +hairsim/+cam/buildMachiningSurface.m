function surface = buildMachiningSurface(haircutState, regionName, headMeshWorld, headQuery, options)
%BUILDMACHININGSURFACE C1 of the camClipper plan: the CAM "workpiece" of
%   one scalp region -- which roots must be machined, the scalp normal and
%   hair-flow direction at each, the region's flow-aligned frame and the
%   head-mesh faces the region's toolpaths may run over.
%
%   haircutState must carry a length goal (hairsim.hair.setUniformTargetLengthGoal
%   or setRegionTargetLengthGoal). A root "needs cutting" while its strand
%   is longer than its target. Flow at a root = the rest-shape strand
%   direction (guide node at mid-length minus root) projected onto the
%   tangent plane -- the direction the hair grows/lies. The clipper feeds
%   AGAINST it (confirmed decision: real clippers are pushed against the
%   growth so the comb lifts the hair).
%
%   Machining boundary: the region's roots; toolpaths may cross into the
%   neighbouring regions by options.MarginM (the region boundary is
%   extended by an overlap band; crossing is harmless because every strand
%   is cut to ITS OWN target).
%
%   surface fields: region, strandIdx (all region strands), needsCut
%   (logical, same order), rootsWorldM, rootNormals, flowDirs, centreWorldM,
%   meanNormal, meanFlow, transverse (= meanNormal x meanFlow), faces
%   (index into headMeshWorld.faces of the region + margin), headQuery.

arguments
    haircutState (1, 1) struct
    regionName (1, 1) string
    headMeshWorld (1, 1) struct
    headQuery (1, 1) struct
    options.MarginM (1, 1) double {mustBeNonnegative} = 0.025
    options.NormalRadiusM (1, 1) double {mustBePositive} = 0.006
end

if ~isfield(haircutState, 'lengthGoal')
    error("hairsim:cam:NoLengthGoal", "Set a length goal on haircutState first.");
end
regions = hairsim.hair.getStrandRegions(haircutState);
strandIdx = find(regions == regionName)';
if isempty(strandIdx)
    error("hairsim:cam:EmptyRegion", "No strands in region %s.", regionName);
end
hairs = haircutState.hairState.hairs;
guides = haircutState.hairState.guideTargets;
roots = [hairs.X(1, strandIdx)', hairs.Y(1, strandIdx)', hairs.Z(1, strandIdx)'];
needsCut = (haircutState.hairState.activeNodeCounts(strandIdx) > ...
    haircutState.lengthGoal.targetActiveNodeCounts(strandIdx))';

rootNormals = hairsim.cam.smoothedNormalAt(roots, headQuery, options.NormalRadiusM);
mid = ceil(size(guides.X, 1) / 2);
growth = [guides.X(mid, strandIdx)', guides.Y(mid, strandIdx)', guides.Z(mid, strandIdx)'] - roots;
flowDirs = growth - sum(growth .* rootNormals, 2) .* rootNormals;
flowDirs = flowDirs ./ max(vecnorm(flowDirs, 2, 2), eps);

meanNormal = mean(rootNormals, 1);
meanNormal = meanNormal / norm(meanNormal);
meanFlow = mean(flowDirs, 1);
meanFlow = meanFlow - dot(meanFlow, meanNormal) * meanNormal;
meanFlow = meanFlow / norm(meanFlow);
transverse = cross(meanNormal, meanFlow);

% faces of the region plus the overlap band: every face within MarginM of a region root
centre = mean(roots, 1);
extent = max(vecnorm(roots - centre, 2, 2)) + options.MarginM + headQuery.maxFaceRadiusM;
nearFaces = find(vecnorm(headQuery.centroids - centre, 2, 2) <= extent);
keep = false(numel(nearFaces), 1);
C = headQuery.centroids(nearFaces, :);
for k = 1:size(roots, 1)
    keep = keep | vecnorm(C - roots(k, :), 2, 2) <= options.MarginM;
end

surface = struct();
surface.region = regionName;
surface.strandIdx = strandIdx;
surface.needsCut = needsCut;
surface.rootsWorldM = roots;
surface.rootNormals = rootNormals;
surface.flowDirs = flowDirs;
surface.centreWorldM = centre;
surface.meanNormal = meanNormal;
surface.meanFlow = meanFlow;
surface.transverse = transverse;
surface.faces = nearFaces(keep);
surface.marginM = options.MarginM;
surface.headQuery = headQuery;
end
