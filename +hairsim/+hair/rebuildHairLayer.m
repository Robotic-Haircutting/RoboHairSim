function hairLayer = rebuildHairLayer(hairLayer, options)
%REBUILDHAIRLAYER Rebuild hair geometry while preserving layer settings.

arguments
    hairLayer (1, 1) struct
    options.HairLengthFactor (1, 1) double {mustBePositive} = hairLayer.creation.hairLengthFactor
    options.HairDensityFactor (1, 1) double {mustBePositive} = hairLayer.creation.hairDensityFactor
end

hairLayer.creation.hairLengthFactor = options.HairLengthFactor;
hairLayer.creation.hairDensityFactor = options.HairDensityFactor;
hairLayer.haircutState.params.rigidity = hairLayer.physics.rigidity;
hairLayer.haircutState.params.color = hairLayer.style.color;
hairLayer.haircutState.params.alpha = hairLayer.style.alpha;
hairLayer.haircutState.params.lineWidth = hairLayer.style.lineWidth;
hairLayer.haircutState = hairsim.hair.rebuildHaircutState(hairLayer.haircutState, ...
    hairLayer.rootsPath, hairLayer.headMeshLocal, ...
    hairLayer.T_world_head, hairLayer.headScaleFactor, ...
    'HairLengthFactor', hairLayer.creation.hairLengthFactor, ...
    'HairDensityFactor', hairLayer.creation.hairDensityFactor);
hairLayer.physics.nextStrand = 1;
hairLayer.physics.batchCounter = 0;
end
