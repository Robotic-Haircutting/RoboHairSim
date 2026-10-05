function haircutState = setUniformTargetLengthGoal(haircutState, targetLengthM, options)
%SETUNIFORMTARGETLENGTHGOAL Attach a uniform remaining-length goal.

arguments
    haircutState (1, 1) struct
    targetLengthM (1, 1) double {mustBeFinite, mustBeNonnegative}
    options.ToleranceM (1, 1) double = nan
end

[cumulativeLengthsM, segmentLengthsM] = computeReferenceCumulativeLengths( ...
    haircutState.hairState.guideTargets);
targetActiveNodeCounts = computeTargetActiveNodeCounts( ...
    cumulativeLengthsM, targetLengthM);

toleranceM = options.ToleranceM;
if ~isfinite(toleranceM)
    positiveSegments = segmentLengthsM(segmentLengthsM > eps);
    if isempty(positiveSegments)
        toleranceM = 0;
    else
        toleranceM = 0.5 * median(positiveSegments(:));
    end
end

goal = struct();
goal.type = "uniform_length";
goal.targetLengthM = targetLengthM;
goal.toleranceM = toleranceM;
goal.targetActiveNodeCounts = targetActiveNodeCounts;
goal.targetRemainingLengthM = lookupLengthsAtActiveCounts( ...
    cumulativeLengthsM, targetActiveNodeCounts);
goal.initialRemainingLengthM = cumulativeLengthsM(end, :);
goal.referenceCumulativeLengthsM = cumulativeLengthsM;

haircutState.lengthGoal = goal;
haircutState.params.targetHairLengthM = targetLengthM;
haircutState.params.targetHairLengthToleranceM = toleranceM;
end

function [cumulativeLengthsM, segmentLengthsM] = computeReferenceCumulativeLengths(guides)
guideX = guides.X;
guideY = guides.Y;
guideZ = guides.Z;
[nodesPerStrand, numStrands] = size(guideX);
cumulativeLengthsM = zeros(nodesPerStrand, numStrands);
segmentLengthsM = zeros(max(nodesPerStrand - 1, 0), numStrands);

for strandIdx = 1:numStrands
    points = [guideX(:, strandIdx), guideY(:, strandIdx), guideZ(:, strandIdx)];
    if nodesPerStrand > 1
        segmentLengths = vecnorm(diff(points, 1, 1), 2, 2);
        segmentLengthsM(:, strandIdx) = segmentLengths;
        cumulativeLengthsM(:, strandIdx) = [0; cumsum(segmentLengths)];
    end
end
end

function targetActiveNodeCounts = computeTargetActiveNodeCounts( ...
        cumulativeLengthsM, targetLengthM)
[nodesPerStrand, numStrands] = size(cumulativeLengthsM);
targetActiveNodeCounts = ones(1, numStrands);
for strandIdx = 1:numStrands
    firstAtOrAboveTarget = find( ...
        cumulativeLengthsM(:, strandIdx) >= targetLengthM, 1, 'first');
    if isempty(firstAtOrAboveTarget)
        targetActiveNodeCounts(strandIdx) = nodesPerStrand;
    else
        targetActiveNodeCounts(strandIdx) = firstAtOrAboveTarget;
    end
end
end

function lengthsM = lookupLengthsAtActiveCounts(cumulativeLengthsM, activeNodeCounts)
numStrands = numel(activeNodeCounts);
lengthsM = zeros(1, numStrands);
for strandIdx = 1:numStrands
    nodeIdx = min(max(activeNodeCounts(strandIdx), 1), size(cumulativeLengthsM, 1));
    lengthsM(strandIdx) = cumulativeLengthsM(nodeIdx, strandIdx);
end
end
