function lengthsM = computeActiveStrandLengths(hairState)
%COMPUTEACTIVESTRANDLENGTHS Compute visible polyline length per guide strand.

hairs = hairState.hairs;
activeNodeCounts = hairState.activeNodeCounts;
numStrands = numel(activeNodeCounts);
lengthsM = zeros(1, numStrands);

for strandIdx = 1:numStrands
    activeCount = min(activeNodeCounts(strandIdx), size(hairs.X, 1));
    if activeCount < 2
        continue;
    end
    points = [
        hairs.X(1:activeCount, strandIdx), ...
        hairs.Y(1:activeCount, strandIdx), ...
        hairs.Z(1:activeCount, strandIdx)
        ];
    lengthsM(strandIdx) = sum(vecnorm(diff(points, 1, 1), 2, 2));
end
end
