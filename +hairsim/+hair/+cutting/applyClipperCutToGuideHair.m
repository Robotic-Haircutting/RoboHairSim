function [activeNodeCounts, cutInfo] = applyClipperCutToGuideHair( ...
        guideX, guideY, guideZ, activeNodeCounts, clipperCenter, clipperRadius, options)
%APPLYCLIPPERCUTTOGUIDEHAIR Shorten guide strands hit by a clipper sphere.

arguments
    guideX (:, :) double
    guideY (:, :) double
    guideZ (:, :) double
    activeNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative}
    clipperCenter (1, 3) double
    clipperRadius (1, 1) double {mustBeNonnegative}
    options.CutRadiusScale (1, 1) double {mustBePositive} = 0.95
    options.TargetActiveNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative} = []
end

hairsim.hair.validateGuideMatrices(guideX, guideY, guideZ, activeNodeCounts);
targetActiveNodeCounts = createTargetActiveNodeCounts( ...
    activeNodeCounts, options.TargetActiveNodeCounts);

cutRadiusM = clipperRadius * options.CutRadiusScale;
cutRadiusSqM = cutRadiusM * cutRadiusM;
strandIndices = [];
nodeIndices = [];

for strandIdx = 1:numel(activeNodeCounts)
    activeCount = min(activeNodeCounts(strandIdx), size(guideX, 1));
    if activeCount < 2
        continue;
    end

    for nodeIdx = 2:activeCount
        dx = guideX(nodeIdx, strandIdx) - clipperCenter(1);
        dy = guideY(nodeIdx, strandIdx) - clipperCenter(2);
        dz = guideZ(nodeIdx, strandIdx) - clipperCenter(3);
        if dx * dx + dy * dy + dz * dz <= cutRadiusSqM
            targetActiveCount = min(targetActiveNodeCounts(strandIdx), activeCount);
            newActiveCount = max(targetActiveCount, nodeIdx - 1);
            if newActiveCount >= activeCount
                break;
            end
            activeNodeCounts(strandIdx) = newActiveCount;
            strandIndices(end + 1) = strandIdx; %#ok<AGROW>
            nodeIndices(end + 1) = newActiveCount + 1; %#ok<AGROW>
            break;
        end
    end
end

cutInfo = struct();
cutInfo.cutCount = numel(strandIndices);
cutInfo.strandIndices = strandIndices;
cutInfo.nodeIndices = nodeIndices;
end

function targetActiveNodeCounts = createTargetActiveNodeCounts( ...
        activeNodeCounts, requestedTargetActiveNodeCounts)
if isempty(requestedTargetActiveNodeCounts)
    targetActiveNodeCounts = zeros(size(activeNodeCounts));
    return;
end
if numel(requestedTargetActiveNodeCounts) ~= numel(activeNodeCounts)
    error("hairsim:GuideHairTargetCountMismatch", ...
        "Target active node count must include one value per guide strand.");
end
targetActiveNodeCounts = requestedTargetActiveNodeCounts;
end
