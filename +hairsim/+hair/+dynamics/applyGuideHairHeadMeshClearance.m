function hairState = applyGuideHairHeadMeshClearance(hairState, headQuery, marginM, options)
%APPLYGUIDEHAIRHEADMESHCLEARANCE Keep dynamic guide hair outside the head
%   mesh: every active node (the root excepted) closer than marginM to the
%   exact head surface (or inside it) is moved to marginM along the surface
%   normal at its closest point. The exact mesh, unlike the convex hull,
%   does not hold the hair away from the scalp where the head is concave
%   (nape, behind the ears).
%
%   headQuery: hairsim.tool.createHeadDistanceQuery of the world head mesh.

arguments
    hairState (1, 1) struct
    headQuery (1, 1) struct
    marginM (1, 1) double {mustBeNonnegative} = 0
    options.StrandIndices (1, :) double {mustBeInteger, mustBePositive} = []
    options.BucketM (1, 1) double {mustBePositive} = 0.03   % nodes are queried per bucket: local face cull
end

[nodesPerStrand, numStrands] = size(hairState.hairs.X);
strandIndices = hairsim.hair.dynamics.createStrandIndexList(numStrands, options.StrandIndices);
counts = min(hairState.activeNodeCounts(strandIndices), nodesPerStrand);
nodeIdx = (2:nodesPerStrand)';
active = nodeIdx <= counts;                           % (K-1) x numel(strandIndices)
[r, c] = find(active);
if isempty(r)
    return;
end
lin = sub2ind([nodesPerStrand, numStrands], nodeIdx(r), strandIndices(c)');
P = [hairState.hairs.X(lin), hairState.hairs.Y(lin), hairState.hairs.Z(lin)];

distanceM = zeros(numel(lin), 1);
closestM = nan(numel(lin), 3);
faceIdx = nan(numel(lin), 1);
capM = max(marginM, 0.001) + 0.01;
[~, ~, bucket] = unique(floor(P / options.BucketM), 'rows');
for b = 1:max(bucket)
    k = find(bucket == b);
    [distanceM(k), closestM(k, :), faceIdx(k)] = hairsim.tool.signedDistanceToHead(P(k, :), headQuery, capM);
end

push = find(distanceM < marginM & ~isnan(closestM(:, 1)));
if isempty(push)
    return;
end
outward = headQuery.normals(faceIdx(push), :);       % face winding is outward
P(push, :) = closestM(push, :) + marginM * outward;
hairState.hairs.X(lin(push)) = P(push, 1);
hairState.hairs.Y(lin(push)) = P(push, 2);
hairState.hairs.Z(lin(push)) = P(push, 3);
end
