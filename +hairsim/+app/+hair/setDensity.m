function appState = setDensity(appState, hairDensityFactor)
%SETDENSITY Ported from run_robot_haircut.m's setHairDensityControl (which
%   called the shared rebuildHairLayerFromControls). Rebuilds the hair
%   geometry at the new density factor, keeping the current length factor.
appState.hairLayer = hairsim.hair.rebuildHairLayer(appState.hairLayer, ...
    'HairLengthFactor', appState.hairLayer.creation.hairLengthFactor, ...
    'HairDensityFactor', hairDensityFactor);
end
