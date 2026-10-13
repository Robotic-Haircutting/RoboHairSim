function uncut = uncutStrands(haircutState)
%UNCUTSTRANDS Which strands still have to be cut: longer than their target
%   (haircutState.lengthGoal, hairsim.app.hair.installClipperCut). With a
%   Region installed, strands outside it have their full length as target,
%   so only the region counts -- the same set the effective-cutting
%   coverage is measured on.
%   uncut fields: idx (strands still above target), numNeeds (strands that
%   needed cutting at all), numDone, rootsWorldM (roots of idx).

arguments
    haircutState (1, 1) struct
end

hairState = haircutState.hairState;
target = haircutState.lengthGoal.targetActiveNodeCounts;
fullCount = size(hairState.guideTargets.X, 1);
needs = target < fullCount;
idx = find(needs & hairState.activeNodeCounts > target);
hairs = hairState.hairs;
uncut = struct('idx', idx, 'numNeeds', nnz(needs), 'numDone', nnz(needs) - numel(idx), ...
    'rootsWorldM', [hairs.X(1, idx)', hairs.Y(1, idx)', hairs.Z(1, idx)']);
end
