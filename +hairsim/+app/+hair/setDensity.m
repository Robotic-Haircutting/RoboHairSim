function appState = setDensity(appState, hairDensityFactor)
%SETDENSITY Set the hair density. Rebuilds the hair
%   geometry at the new density factor, keeping the current length factor.
appState.hairLayer = hairsim.hair.rebuildHairLayer(appState.hairLayer, ...
    'HairLengthFactor', appState.hairLayer.creation.hairLengthFactor, ...
    'HairDensityFactor', hairDensityFactor);
end
