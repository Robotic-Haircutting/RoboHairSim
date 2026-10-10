function [hairLayer, updateInfo] = updateHairLayerForClipper( ...
        hairLayer, T_world_ee, cutFlag, headHullWorld, floorZ, options)
%UPDATEHAIRLAYERFORCLIPPER Advance cutting, physics, and debris for one frame.
%   T_world_ee: the end-effector pose this frame. Cutting needs the clipper
%   cut model in hairLayer.cutting.model (hairsim.app.hair.installClipperCut
%   or hairsim.hair.cutting.createCombLiftCutModelForPlanner).
arguments
    hairLayer (1, 1) struct
    T_world_ee (4, 4) double
    cutFlag (1, 1) logical
    headHullWorld (1, 1) struct
    floorZ (1, 1) double {mustBeFinite}
    options.DebrisRandomScale (1, 1) double {mustBeNonnegative} = 1.0
    % true: physics + debris only, cutting untouched (an idle frame, the
    % robot did not move: a comb-lift sweep in progress is NOT broken)
    options.SkipCutting (1, 1) logical = false
end
cutModel = [];
if isfield(hairLayer.cutting, 'model') && ~options.SkipCutting
    cutModel = hairLayer.cutting.model;
end
if cutFlag && ~options.SkipCutting && isempty(cutModel)
    error("hairsim:NoCutModel", ...
        "No clipper cut model installed (hairsim.app.hair.installClipperCut): nothing can be cut.");
end
updateInfo = struct( ...
    'didCut', false, ...
    'cutCount', 0, ...
    'cutInfo', struct('cutCount', 0, 'strandIndices', [], 'nodeIndices', []), ...
    'didPhysics', false, ...
    'didDebris', false, ...
    'shouldRender', false, ...
    'statusLabel', "ACCEPTED");

isCombLift = ~isempty(cutModel) && cutModel.type == "combLift";
if isCombLift && ~cutFlag
    % a combLift sweep only connects consecutive CUT ticks
    hairLayer.cutting.model.lastToolTransform = [];
end
if cutFlag && ~options.SkipCutting
    previousToolTransform = [];
    if isCombLift
        previousToolTransform = cutModel.lastToolTransform;
    end
    [hairLayer.haircutState, cutInfo] = hairsim.hair.cutting.applyClipperHaircutStep( ...
        hairLayer.haircutState, T_world_ee, cutModel, ...
        'DebrisRandomScale', options.DebrisRandomScale, ...
        'PreviousToolTransform', previousToolTransform);
    if isCombLift
        hairLayer.cutting.model.lastToolTransform = T_world_ee;
    end
    updateInfo.didCut = cutInfo.cutCount > 0;
    updateInfo.cutCount = cutInfo.cutCount;
    updateInfo.cutInfo = cutInfo;
    updateInfo.shouldRender = true;
    if cutInfo.cutCount > 0
        updateInfo.statusLabel = "ACCEPTED:CUT_" + string(cutInfo.cutCount);
    end
end

[strandIndices, hairLayer.physics] = createPhysicsBatchIndices( ...
    hairLayer.haircutState, hairLayer.physics);
if ~isempty(strandIndices)
    hairLayer.haircutState.params.gravity = hairLayer.physics.gravity;
    hairLayer.haircutState.params.rigidity = hairLayer.physics.rigidity;
    hairLayer.haircutState.params.damping = hairLayer.physics.damping;
    hairLayer.haircutState.params.dt = hairLayer.physics.dt;
    % the hair collides with the exact head mesh when the layer carries its
    % query (hairsim.hair.createHairLayer), else with the head hull
    headCollision = headHullWorld;
    if isfield(hairLayer.physics, 'headQuery') && ~isempty(hairLayer.physics.headQuery)
        headCollision = hairLayer.physics.headQuery;
    end
    iterations = 10;
    if isfield(hairLayer.physics, 'constraintIterations')
        iterations = hairLayer.physics.constraintIterations;
    end
    hairLayer.haircutState = hairsim.hair.dynamics.stepHaircutPhysics( ...
        hairLayer.haircutState, headCollision, ...
        'HeadClearanceM', hairLayer.physics.headClearanceM, ...
        'ConstraintIterations', iterations, ...
        'StrandIndices', strandIndices);
    hairLayer.physics.batchCounter = hairLayer.physics.batchCounter + 1;
    updateInfo.didPhysics = true;
    if mod(hairLayer.physics.batchCounter, hairLayer.physics.renderBatchStride) == 0 || ...
            hairLayer.physics.nextStrand == 1
        updateInfo.shouldRender = true;
        if updateInfo.statusLabel == "ACCEPTED"
            updateInfo.statusLabel = "PHYSICS";
        end
    end
end

if ~isempty(hairLayer.haircutState.debrisList)
    hairLayer.haircutState = hairsim.hair.debris.stepGuideHairDebris( ...
        hairLayer.haircutState, floorZ);
    updateInfo.didDebris = true;
    updateInfo.shouldRender = true;
    if updateInfo.statusLabel == "ACCEPTED" || updateInfo.statusLabel == "PHYSICS"
        updateInfo.statusLabel = "DEBRIS";
    end
end
end

function [strandIndices, physics] = createPhysicsBatchIndices(haircutState, physics)
numStrands = numel(haircutState.hairState.activeNodeCounts);
if numStrands == 0
    strandIndices = [];
    return;
end

batchEnd = min(numStrands, physics.nextStrand + physics.batchSize - 1);
strandIndices = physics.nextStrand:batchEnd;
physics.nextStrand = batchEnd + 1;
if physics.nextStrand > numStrands
    physics.nextStrand = 1;
end
end
