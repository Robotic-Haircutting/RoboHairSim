function appState = setLength(appState, hairLengthFactor)
%SETLENGTH Ported from run_robot_haircut.m's setHairLengthControl (which
%   called the shared rebuildHairLayerFromControls). Rebuilds the hair
%   geometry at the new length factor, keeping the current density factor
%   (hairsim.hair.rebuildHairLayer itself re-applies the current
%   rigidity/style fields from hairLayer, so nothing else needs restating
%   here -- see hairsim.app.hair.createHairAppState's note).
appState.hairLayer = hairsim.hair.rebuildHairLayer(appState.hairLayer, ...
    'HairLengthFactor', hairLengthFactor, ...
    'HairDensityFactor', appState.hairLayer.creation.hairDensityFactor);
end
