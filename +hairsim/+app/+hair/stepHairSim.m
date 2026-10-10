function [appState, hairStepResult] = stepHairSim(appState, T_world_ee, cutFlag, headHullWorld, floorZ, options)
%STEPHAIRSIM Advance cutting/physics/debris one frame; thin wrapper around
%   hairsim.hair.updateHairLayerForClipper.
%
%   hairStepResult (= the existing updateInfo shape): cutInfo, shouldRender,
%   statusLabel (didCut/cutCount/didPhysics/didDebris also present,
%   unchanged). The caller (App shell) decides what to do with
%   shouldRender/statusLabel -- this function never touches an axes or a
%   recording log itself.
arguments
    appState (1, 1) struct
    T_world_ee (4, 4) double
    cutFlag (1, 1) logical
    headHullWorld (1, 1) struct
    floorZ (1, 1) double {mustBeFinite}
    options.SkipCutting (1, 1) logical = false   % idle frame: physics + debris only
end
[appState.hairLayer, hairStepResult] = hairsim.hair.updateHairLayerForClipper( ...
    appState.hairLayer, T_world_ee, cutFlag, headHullWorld, floorZ, ...
    'SkipCutting', options.SkipCutting);
end
