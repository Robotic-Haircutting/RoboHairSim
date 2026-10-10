function [haircutState, cutInfo] = applyClipperHaircutStep( ...
        haircutState, T_world_ee, cutModel, options)
%APPLYCLIPPERHAIRCUTSTEP Shorten hit guide hair and append falling debris.
%   T_world_ee: the end-effector pose of this CUT tick. cutModel is the
%   clipper's cut model, "combLift" (hairsim.hair.cutting.createCombLiftCutModel):
%   the comb stands covered strands up, cut at lengthGoal's target; the same
%   predicate the planner's coverage uses. PreviousToolTransform (the last
%   CUT tick's T_world_ee) makes it sweep between ticks.
arguments
    haircutState (1, 1) struct
    T_world_ee (4, 4) double
    cutModel (1, 1) struct
    options.DebrisRandomScale (1, 1) double {mustBeNonnegative} = 0.45
    options.PreviousToolTransform = []
end
previousActiveHairNodeCounts = haircutState.hairState.activeNodeCounts;
if cutModel.type == "combLift"
    targetActiveNodeCounts = getTargetActiveNodeCounts(haircutState);
    if isempty(targetActiveNodeCounts)
        error("hairsim:CombLiftNeedsLengthGoal", ...
            "The combLift cut model cuts to lengthGoal.targetActiveNodeCounts; set a length goal first.");
    end
    T_ee = T_world_ee;
    if ~isempty(options.PreviousToolTransform)
        T_ee = cat(3, options.PreviousToolTransform, T_world_ee);
    end
    T_world_tcp = pagemtimes(T_ee, cutModel.tool.T_ee_tcp);
    flowDirs = zeros(0, 3);
    if isfield(cutModel, 'flowDirs')
        flowDirs = cutModel.flowDirs;
    end
    [haircutState.hairState.activeNodeCounts, cutInfo] = ...
        hairsim.hair.cutting.applyCombLiftCutToGuideHair( ...
            haircutState.hairState.hairs.X, ...
            haircutState.hairState.hairs.Y, ...
            haircutState.hairState.hairs.Z, ...
            haircutState.hairState.activeNodeCounts, ...
            targetActiveNodeCounts, T_world_tcp, cutModel.tool, ...
            'FlowDirs', flowDirs);
else
    error("hairsim:UnknownCutModel", "Unknown cut model type '%s'.", cutModel.type);
end

if cutInfo.cutCount > 0
    newDebrisList = hairsim.hair.debris.createGuideHairDebrisSegments( ...
        haircutState.hairState.hairs.X, ...
        haircutState.hairState.hairs.Y, ...
        haircutState.hairState.hairs.Z, ...
        cutInfo, previousActiveHairNodeCounts, haircutState.params.debrisGravity, ...
        'RandomScale', options.DebrisRandomScale);
    haircutState.debrisList = [haircutState.debrisList, newDebrisList];
end

haircutState.lineData = hairsim.hair.render.createHaircutLineData( ...
    haircutState.hairState, haircutState.referenceHairNodeCounts, ...
    haircutState.debrisList);
end

function targetActiveNodeCounts = getTargetActiveNodeCounts(haircutState)
targetActiveNodeCounts = [];
if isfield(haircutState, 'lengthGoal') && ...
        isfield(haircutState.lengthGoal, 'targetActiveNodeCounts')
    targetActiveNodeCounts = haircutState.lengthGoal.targetActiveNodeCounts;
end
end
