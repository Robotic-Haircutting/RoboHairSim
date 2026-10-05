function [activeNodeCounts, cutInfo] = applyGuardBladeCutToGuideHair( ...
        guideX, guideY, guideZ, activeNodeCounts, T_world_ee, cutModel)
%APPLYGUARDBLADECUTTOGUIDEHAIR Flat clipper blade + guard comb cut.
%   Purely geometric: uses ONLY the tool pose and the scalp surface, never
%   any per-strand target (no lengthGoal.targetActiveNodeCounts) -- i.e.
%   only information a real robot holding a clipper would have.
%
%   A guide node is cut when BOTH hold:
%     1. it lies inside the blade's cutting box, expressed in the EE frame
%        (x = travel, y = blade width, z = toward the scalp):
%          |x| <= bladeHalfDepthM, |y| <= bladeHalfWidthM,
%          bladeTipZM - bladeHeightM <= z <= bladeTipZM
%        (the blade plane is at z = bladeTipZM; nothing between the blade
%        and the scalp can be cut);
%     2. its height above the scalp is > guardHeightM (a guard comb of that
%        height: the target contour = scalp offset outward by guardHeightM;
%        everything inside the contour is never cut, so the scalp can never
%        be cut bald below guardHeightM).
%   Each hit strand is cut at its first (root-most) qualifying node, the
%   same per-strand rule as hairsim.hair.cutting.applyClipperCutToGuideHair.
%
%   cutModel (hairsim.hair.cutting.createGuardBladeCutModel): bladeHalfWidthM,
%   bladeHalfDepthM, bladeHeightM, bladeTipZM, guardHeightM, scalpTri
%   (triangulation), scalpVertices, scalpOutwardNormals.

arguments
    guideX (:, :) double
    guideY (:, :) double
    guideZ (:, :) double
    activeNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative}
    T_world_ee (4, 4) double
    cutModel (1, 1) struct
end

hairsim.hair.validateGuideMatrices(guideX, guideY, guideZ, activeNodeCounts);
R = T_world_ee(1:3, 1:3);
p = T_world_ee(1:3, 4)';
nodesPerStrand = size(guideX, 1);

% ---- candidate nodes: active, not the root, inside the blade box
nodeIdx = (1:nodesPerStrand)';
activeMask = nodeIdx >= 2 & nodeIdx <= min(activeNodeCounts, nodesPerStrand);
worldPoints = [guideX(activeMask), guideY(activeMask), guideZ(activeMask)];
localPoints = (worldPoints - p) * R;
inBox = abs(localPoints(:, 1)) <= cutModel.bladeHalfDepthM & ...
    abs(localPoints(:, 2)) <= cutModel.bladeHalfWidthM & ...
    localPoints(:, 3) <= cutModel.bladeTipZM & ...
    localPoints(:, 3) >= cutModel.bladeTipZM - cutModel.bladeHeightM;

hitMask = false(size(activeMask));
activeLinear = find(activeMask);
candidateLinear = activeLinear(inBox);
if ~isempty(candidateLinear)
    candidatePoints = worldPoints(inBox, :);
    nearest = nearestNeighbor(cutModel.scalpTri, candidatePoints);
    heightM = sum((candidatePoints - cutModel.scalpVertices(nearest, :)) .* ...
        cutModel.scalpOutwardNormals(nearest, :), 2);
    hitMask(candidateLinear(heightM > cutModel.guardHeightM)) = true;
end

strandIndices = [];
nodeIndices = [];
for strandIdx = find(any(hitMask, 1))
    firstHit = find(hitMask(:, strandIdx), 1, 'first');
    activeNodeCounts(strandIdx) = firstHit - 1;
    strandIndices(end + 1) = strandIdx; %#ok<AGROW>
    nodeIndices(end + 1) = firstHit; %#ok<AGROW>
end

cutInfo = struct();
cutInfo.cutCount = numel(strandIndices);
cutInfo.strandIndices = strandIndices;
cutInfo.nodeIndices = nodeIndices;
end
