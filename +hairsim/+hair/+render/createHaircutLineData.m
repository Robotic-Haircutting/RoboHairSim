function lineData = createHaircutLineData(hairState, referenceHairNodeCounts, debrisList)
%CREATEHAIRCUTLINEDATA Build plot-ready line data for active hair and debris.

arguments
    hairState (1, 1) struct
    referenceHairNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative}
    debrisList (1, :) struct = struct('X', {}, 'Y', {}, 'Z', {}, ...
        'vx', {}, 'vy', {}, 'vz', {}, 'isStopped', {}, 'stopAge', {})
end

cutStateLineData = hairsim.hair.render.createGuideHairCutStateLineData( ...
    hairState.hairs.X, hairState.hairs.Y, hairState.hairs.Z, ...
    hairState.activeNodeCounts, referenceHairNodeCounts);
[debrisX, debrisY, debrisZ] = ...
    hairsim.hair.render.createGuideHairDebrisLineData(debrisList);
[hairX, hairY, hairZ] = combineHairLineData( ...
    cutStateLineData.uncutX, cutStateLineData.uncutY, cutStateLineData.uncutZ, ...
    cutStateLineData.cutX, cutStateLineData.cutY, cutStateLineData.cutZ);

lineData = struct();
lineData.uncutX = cutStateLineData.uncutX;
lineData.uncutY = cutStateLineData.uncutY;
lineData.uncutZ = cutStateLineData.uncutZ;
lineData.cutX = cutStateLineData.cutX;
lineData.cutY = cutStateLineData.cutY;
lineData.cutZ = cutStateLineData.cutZ;
lineData.hairX = hairX;
lineData.hairY = hairY;
lineData.hairZ = hairZ;
lineData.debrisX = debrisX;
lineData.debrisY = debrisY;
lineData.debrisZ = debrisZ;
end

function [combinedX, combinedY, combinedZ] = combineHairLineData( ...
        lineX, lineY, lineZ, cutLineX, cutLineY, cutLineZ)
combinedX = zeros(0, 1);
combinedY = zeros(0, 1);
combinedZ = zeros(0, 1);

if ~isempty(lineX)
    combinedX = [combinedX; lineX(:)];
    combinedY = [combinedY; lineY(:)];
    combinedZ = [combinedZ; lineZ(:)];
end

if ~isempty(cutLineX)
    if ~isempty(combinedX)
        combinedX = [combinedX; nan];
        combinedY = [combinedY; nan];
        combinedZ = [combinedZ; nan];
    end
    combinedX = [combinedX; cutLineX(:)];
    combinedY = [combinedY; cutLineY(:)];
    combinedZ = [combinedZ; cutLineZ(:)];
end
end
