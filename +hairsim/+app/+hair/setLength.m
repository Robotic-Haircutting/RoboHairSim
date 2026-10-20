function appState = setLength(appState, hairLengthFactor)
%SETLENGTH Set the hair length. Rebuilds the hair
%   geometry at the new length factor, keeping the current density factor
%   (hairsim.hair.rebuildHairLayer itself re-applies the current
%   rigidity/style fields from hairLayer, so nothing else needs restating
%   here -- see hairsim.app.hair.createHairAppState's note).
appState.hairLayer = hairsim.hair.rebuildHairLayer(appState.hairLayer, ...
    'HairLengthFactor', hairLengthFactor, ...
    'HairDensityFactor', appState.hairLayer.creation.hairDensityFactor);
end
