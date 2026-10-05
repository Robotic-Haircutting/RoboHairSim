function strandIndices = createStrandIndexList(numStrands, requestedStrandIndices)
%CREATESTRANDINDEXLIST Resolve which strands to iterate: all, or a requested subset.
%   Shared by hairsim.hair.dynamics.applyGuideHairHeadHullClearance and
%   hairsim.hair.dynamics.stepGuideHairPhysics.

if isempty(requestedStrandIndices)
    strandIndices = 1:numStrands;
    return;
end
strandIndices = unique(requestedStrandIndices(requestedStrandIndices <= numStrands), 'stable');
end
