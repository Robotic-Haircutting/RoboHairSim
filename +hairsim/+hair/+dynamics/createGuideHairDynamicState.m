function hairState = createGuideHairDynamicState(guideX, guideY, guideZ, options)
%CREATEGUIDEHAIRDYNAMICSTATE Initialize dynamic guide-hair simulation state.

arguments
    guideX (:, :) double
    guideY (:, :) double
    guideZ (:, :) double
    options.ActiveNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative} = []
    options.GuideAttraction (:, :) double = []
    options.GravityWeight (:, :) double = []
    options.BendSoftness (:, :) double = []
end

validateGuideMatrices(guideX, guideY, guideZ);
[nodesPerStrand, numStrands] = size(guideX);

hairState = struct();
hairState.hairs = struct('X', guideX, 'Y', guideY, 'Z', guideZ);
hairState.guideTargets = struct('X', guideX, 'Y', guideY, 'Z', guideZ);
hairState.vel = struct( ...
    'X', zeros(nodesPerStrand, numStrands), ...
    'Y', zeros(nodesPerStrand, numStrands), ...
    'Z', zeros(nodesPerStrand, numStrands));

if isempty(options.ActiveNodeCounts)
    hairState.activeNodeCounts = repmat(nodesPerStrand, 1, numStrands);
else
    if numel(options.ActiveNodeCounts) ~= numStrands
        error("hairsim:GuideHairActiveCountMismatch", ...
            "Active node count must include one value per guide strand.");
    end
    hairState.activeNodeCounts = options.ActiveNodeCounts;
end

hairState.guideAttraction = defaultMatrix(options.GuideAttraction, ...
    ones(nodesPerStrand, numStrands));
hairState.gravityWeight = defaultMatrix(options.GravityWeight, ...
    ones(nodesPerStrand, numStrands));
hairState.bendSoftness = defaultMatrix(options.BendSoftness, ...
    zeros(nodesPerStrand, numStrands));
hairState.targetSegmentLengths = computeTargetSegmentLengths(guideX, guideY, guideZ);
end

function validateGuideMatrices(guideX, guideY, guideZ)
if ~isequal(size(guideX), size(guideY)) || ~isequal(size(guideX), size(guideZ))
    error("hairsim:GuideHairSizeMismatch", ...
        "Guide hair X, Y, and Z matrices must have matching sizes.");
end
end

function matrixValue = defaultMatrix(candidate, defaultValue)
if isempty(candidate)
    matrixValue = defaultValue;
    return;
end
if ~isequal(size(candidate), size(defaultValue))
    error("hairsim:GuideHairWeightSizeMismatch", ...
        "Guide hair weight matrices must match guide matrix size.");
end
matrixValue = candidate;
end

function targetSegmentLengths = computeTargetSegmentLengths(guideX, guideY, guideZ)
numStrands = size(guideX, 2);
targetSegmentLengths = zeros(1, numStrands);
for strandIdx = 1:numStrands
    segmentVectors = diff([guideX(:, strandIdx), guideY(:, strandIdx), guideZ(:, strandIdx)], 1, 1);
    segmentLengths = vecnorm(segmentVectors, 2, 2);
    positiveLengths = segmentLengths(segmentLengths > eps);
    if isempty(positiveLengths)
        targetSegmentLengths(strandIdx) = 0;
    else
        targetSegmentLengths(strandIdx) = mean(positiveLengths);
    end
end
end
