function validateGuideMatrices(guideX, guideY, guideZ, activeNodeCounts)
%VALIDATEGUIDEMATRICES Check guide-hair X/Y/Z matrices and active-count shapes match.
%   Shared by hairsim.hair.cutting.applyClipperCutToGuideHair and
%   hairsim.hair.debris.createGuideHairDebrisSegments.

if ~isequal(size(guideX), size(guideY)) || ~isequal(size(guideX), size(guideZ))
    error("hairsim:GuideHairSizeMismatch", ...
        "Guide hair X, Y, and Z matrices must have matching sizes.");
end
if numel(activeNodeCounts) ~= size(guideX, 2)
    error("hairsim:GuideHairActiveCountMismatch", ...
        "Active node count must include one value per guide strand.");
end
end
