function [lineX, lineY, lineZ] = createGuideHairDebrisLineData(debrisList)
%CREATEGUIDEHAIRDEBRISLINEDATA Build plot line data for hair debris.

arguments
    debrisList (1, :) struct
end

lineLength = 0;
for debrisIdx = 1:numel(debrisList)
    lineLength = lineLength + numel(debrisList(debrisIdx).X) + 1;
end

lineX = nan(lineLength, 1);
lineY = nan(lineLength, 1);
lineZ = nan(lineLength, 1);
cursor = 1;
for debrisIdx = 1:numel(debrisList)
    debris = debrisList(debrisIdx);
    segmentLength = numel(debris.X);
    segmentRange = cursor:(cursor + segmentLength - 1);
    lineX(segmentRange) = debris.X(:);
    lineY(segmentRange) = debris.Y(:);
    lineZ(segmentRange) = debris.Z(:);
    cursor = cursor + segmentLength + 1;
end
end
