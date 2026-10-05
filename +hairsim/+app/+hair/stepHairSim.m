function [appState, hairStepResult] = stepHairSim(appState, clipperCenterWorldM, toolRadiusM, cutFlag, headHullWorld, floorZ, options)
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
    clipperCenterWorldM (1, 3) double {mustBeFinite}
    toolRadiusM (1, 1) double {mustBePositive}
    cutFlag (1, 1) logical
    headHullWorld (1, 1) struct
    floorZ (1, 1) double {mustBeFinite}
    options.ToolTransform = []
    options.SkipCutting (1, 1) logical = false   % idle frame: physics + debris only
end
[appState.hairLayer, hairStepResult] = hairsim.hair.updateHairLayerForClipper( ...
    appState.hairLayer, clipperCenterWorldM, toolRadiusM, cutFlag, headHullWorld, floorZ, ...
    'ToolTransform', options.ToolTransform, 'SkipCutting', options.SkipCutting);
end
