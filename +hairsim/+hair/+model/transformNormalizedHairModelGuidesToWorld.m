function [guideXWorld, guideYWorld, guideZWorld] = transformNormalizedHairModelGuidesToWorld( ...
        hairModel, headMeshLocal, T_world_head, meshScaleFactor)
%TRANSFORMNORMALIZEDHAIRMODELGUIDESTOWORLD Map normalized guide matrices to world.

arguments
    hairModel (1, 1) struct
    headMeshLocal (1, 1) struct
    T_world_head (4, 4) double {mustBeFinite}
    meshScaleFactor (1, 1) double {mustBePositive} = 1.0
end

normalizedPoints = [hairModel.guideX(:), hairModel.guideY(:), hairModel.guideZ(:)];
worldPoints = transformNormalizedPointsToWorld( ...
    normalizedPoints, headMeshLocal, T_world_head, meshScaleFactor);

guideXWorld = reshape(worldPoints(:, 1), size(hairModel.guideX));
guideYWorld = reshape(worldPoints(:, 2), size(hairModel.guideY));
guideZWorld = reshape(worldPoints(:, 3), size(hairModel.guideZ));
end

function worldPoints = transformNormalizedPointsToWorld( ...
        normalizedPoints, headMeshLocal, T_world_head, meshScaleFactor)
normalizedScaleM = max(headMeshLocal.bboxSize) / 2;
localPointsM = headMeshLocal.center + normalizedPoints * normalizedScaleM;
worldPoints = (localPointsM * meshScaleFactor) * ...
    T_world_head(1:3, 1:3)' + T_world_head(1:3, 4)';
end
