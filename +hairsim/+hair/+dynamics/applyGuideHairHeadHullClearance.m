function hairState = applyGuideHairHeadHullClearance(hairState, hullWorld, marginM, options)
%APPLYGUIDEHAIRHEADHULLCLEARANCE Keep dynamic guide hair outside head hull.

arguments
    hairState (1, 1) struct
    hullWorld (1, 1) struct
    marginM (1, 1) double {mustBeNonnegative} = 0
    options.StrandIndices (1, :) double {mustBeInteger, mustBePositive} = []
end

[nodesPerStrand, numStrands] = size(hairState.hairs.X);
strandIndices = hairsim.hair.dynamics.createStrandIndexList(numStrands, options.StrandIndices);
for strandIdx = strandIndices
    activeCount = min(hairState.activeNodeCounts(strandIdx), nodesPerStrand);
    if activeCount < 1
        continue;
    end

    hairState = hairsim.hair.dynamics.pinRootToGuide(hairState, strandIdx);
    for nodeIdx = 2:activeCount
        point = [ ...
            hairState.hairs.X(nodeIdx, strandIdx), ...
            hairState.hairs.Y(nodeIdx, strandIdx), ...
            hairState.hairs.Z(nodeIdx, strandIdx)];
        [clearanceM, activePlaneIndex] = headHullClearance(point, hullWorld);
        if clearanceM >= marginM
            continue;
        end

        normal = hullWorld.normalsWorld(activePlaneIndex, :);
        correction = (marginM - clearanceM) * normal;
        point = point + correction;
        hairState.hairs.X(nodeIdx, strandIdx) = point(1);
        hairState.hairs.Y(nodeIdx, strandIdx) = point(2);
        hairState.hairs.Z(nodeIdx, strandIdx) = point(3);

        velocity = [ ...
            hairState.vel.X(nodeIdx, strandIdx), ...
            hairState.vel.Y(nodeIdx, strandIdx), ...
            hairState.vel.Z(nodeIdx, strandIdx)];
        inwardVelocity = dot(velocity, normal);
        if inwardVelocity < 0
            velocity = velocity - inwardVelocity * normal;
            hairState.vel.X(nodeIdx, strandIdx) = velocity(1);
            hairState.vel.Y(nodeIdx, strandIdx) = velocity(2);
            hairState.vel.Z(nodeIdx, strandIdx) = velocity(3);
        end
    end
end
end

function [clearanceM, activePlaneIndex] = headHullClearance(pointWorldM, hullWorld)
signedDistancesM = pointWorldM * hullWorld.normalsWorld' + hullWorld.offsetsWorldM';
[clearanceM, activePlaneIndex] = max(signedDistancesM);
end
