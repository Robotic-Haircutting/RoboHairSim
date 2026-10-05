function strandRegions = getStrandRegions(haircutState)
%GETSTRANDREGIONS Per-strand scalp region labels ("scalp" if unavailable).

numStrands = numel(haircutState.hairState.activeNodeCounts);
strandRegions = repmat("scalp", numStrands, 1);
if ~isfield(haircutState, 'hairModelLocal') || ...
        ~isfield(haircutState.hairModelLocal, 'regions')
    return;
end

availableRegions = string(haircutState.hairModelLocal.regions(:));
copyCount = min(numStrands, numel(availableRegions));
strandRegions(1:copyCount) = availableRegions(1:copyCount);
end
