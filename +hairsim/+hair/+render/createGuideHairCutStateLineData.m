function lineData = createGuideHairCutStateLineData( ...
        guideX, guideY, guideZ, activeNodeCounts, referenceNodeCounts)
%CREATEGUIDEHAIRCUTSTATELINEDATA Split visible guide hair by cut state.

arguments
    guideX (:, :) double
    guideY (:, :) double
    guideZ (:, :) double
    activeNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative}
    referenceNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative}
end

validateGuideMatrices(guideX, guideY, guideZ, activeNodeCounts, referenceNodeCounts);

activeNodeCounts = min(activeNodeCounts(:)', size(guideX, 1));
referenceNodeCounts = min(referenceNodeCounts(:)', size(guideX, 1));
isCutStrand = activeNodeCounts < referenceNodeCounts;

[lineData.uncutX, lineData.uncutY, lineData.uncutZ] = buildLineDataForStrands( ...
    guideX, guideY, guideZ, activeNodeCounts, ~isCutStrand);
[lineData.cutX, lineData.cutY, lineData.cutZ] = buildLineDataForStrands( ...
    guideX, guideY, guideZ, activeNodeCounts, isCutStrand);
end

function validateGuideMatrices(guideX, guideY, guideZ, activeNodeCounts, referenceNodeCounts)
if ~isequal(size(guideX), size(guideY)) || ~isequal(size(guideX), size(guideZ))
    error("hairsim:GuideHairSizeMismatch", ...
        "Guide hair X, Y, and Z matrices must have matching sizes.");
end
if numel(activeNodeCounts) ~= size(guideX, 2) || ...
        numel(referenceNodeCounts) ~= size(guideX, 2)
    error("hairsim:GuideHairActiveCountMismatch", ...
        "Active and reference node counts must include one value per guide strand.");
end
end

function [lineX, lineY, lineZ] = buildLineDataForStrands( ...
        guideX, guideY, guideZ, activeNodeCounts, strandMask)
lineLength = sum(activeNodeCounts(strandMask) + 1);
lineX = nan(lineLength, 1);
lineY = nan(lineLength, 1);
lineZ = nan(lineLength, 1);

cursor = 1;
for strandIdx = find(strandMask)
    activeCount = activeNodeCounts(strandIdx);
    if activeCount > 0
        strandRange = cursor:(cursor + activeCount - 1);
        lineX(strandRange) = guideX(1:activeCount, strandIdx);
        lineY(strandRange) = guideY(1:activeCount, strandIdx);
        lineZ(strandRange) = guideZ(1:activeCount, strandIdx);
    end
    cursor = cursor + activeCount + 1;
end
end
