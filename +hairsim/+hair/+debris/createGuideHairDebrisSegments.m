function debrisList = createGuideHairDebrisSegments( ...
        guideX, guideY, guideZ, cutInfo, previousActiveNodeCounts, gravity, options)
%CREATEGUIDEHAIRDEBRISSEGMENTS Create falling debris for newly cut guide hair.

arguments
    guideX (:, :) double
    guideY (:, :) double
    guideZ (:, :) double
    cutInfo (1, 1) struct
    previousActiveNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative}
    gravity (1, 3) double
    options.RandomScale (1, 1) double {mustBeNonnegative} = 0.45
end

hairsim.hair.validateGuideMatrices(guideX, guideY, guideZ, previousActiveNodeCounts);

debrisList = struct('X', {}, 'Y', {}, 'Z', {}, ...
    'vx', {}, 'vy', {}, 'vz', {}, 'isStopped', {}, 'stopAge', {});
if ~isfield(cutInfo, 'cutCount') || cutInfo.cutCount <= 0
    return;
end

for cutIdx = 1:cutInfo.cutCount
    strandIdx = cutInfo.strandIndices(cutIdx);
    nodeIdx = cutInfo.nodeIndices(cutIdx);
    strandEndIdx = min(previousActiveNodeCounts(strandIdx), size(guideX, 1));
    if nodeIdx > strandEndIdx
        continue;
    end

    nodeRange = nodeIdx:strandEndIdx;
    segmentLength = numel(nodeRange);
    debris = struct();
    debris.X = guideX(nodeRange, strandIdx);
    debris.Y = guideY(nodeRange, strandIdx);
    debris.Z = guideZ(nodeRange, strandIdx);
    debris.vx = (rand(segmentLength, 1) - 0.5) * 0.01 * options.RandomScale;
    debris.vy = (rand(segmentLength, 1) - 0.5) * 0.01 * options.RandomScale;
    debris.vz = ones(segmentLength, 1) * gravity(3) * 0.5;
    debris.isStopped = false(segmentLength, 1);
    debris.stopAge = 0;
    debrisList(end + 1) = debris; %#ok<AGROW>
end
end
