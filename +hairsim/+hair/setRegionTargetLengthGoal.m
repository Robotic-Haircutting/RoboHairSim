function haircutState = setRegionTargetLengthGoal(haircutState, regionLengthsM, defaultLengthM)
%SETREGIONTARGETLENGTHGOAL Per-region remaining-length goal (guard-comb
%   length per scalp region). Built from hairsim.hair.setUniformTargetLengthGoal
%   once per distinct length, then merged strand by strand, so a region set
%   to L gets exactly the targets a uniform goal of L would give it.
%
%   regionLengthsM: struct, field = region name (e.g. scalp_back), value =
%   target length in metres. Regions not listed use defaultLengthM.
%   The result keeps lengthGoal.type = "uniform_length" (every consumer,
%   e.g. hairsim.hair.findTargetCutNodes, reads targetActiveNodeCounts per
%   strand and works unchanged) and adds lengthGoal.regionLengthsM and
%   lengthGoal.targetLengthPerStrandM.

arguments
    haircutState (1, 1) struct
    regionLengthsM (1, 1) struct
    defaultLengthM (1, 1) double {mustBeFinite, mustBeNonnegative}
end

regions = hairsim.hair.getStrandRegions(haircutState);
lengthPerStrand = repmat(defaultLengthM, 1, numel(regions));
names = fieldnames(regionLengthsM);
for k = 1:numel(names)
    lengthPerStrand(regions == string(names{k})) = regionLengthsM.(names{k});
end

merged = hairsim.hair.setUniformTargetLengthGoal(haircutState, defaultLengthM);
goal = merged.lengthGoal;
for L = unique(lengthPerStrand)
    if L == defaultLengthM
        continue;
    end
    other = hairsim.hair.setUniformTargetLengthGoal(haircutState, L).lengthGoal;
    mask = lengthPerStrand == L;
    goal.targetActiveNodeCounts(mask) = other.targetActiveNodeCounts(mask);
    goal.targetRemainingLengthM(mask) = other.targetRemainingLengthM(mask);
end
goal.regionLengthsM = regionLengthsM;
goal.targetLengthPerStrandM = lengthPerStrand;
haircutState.lengthGoal = goal;
end
