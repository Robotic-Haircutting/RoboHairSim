function [planningState, planResult] = planHaircut(planningState, planRequest)
%PLANHAIRCUT Plan one region, with disk caching. Thin wrapper around
%   hairsim.api.planRegion (itself a thin wrapper around
%   hairsim.planning.getRegionPlanner) plus hairsim.motion.regionPlanCache
%   -- this is the entire subsystem; no planning logic lives here. Ported
%   from run_robot_haircut.m's planAutoRegion, minus the interactive
%   questdlg prompt: that is a UI concern, so this function just reports
%   planResult.fromCache/cacheReason and lets the App shell decide whether
%   to ask the user or use the cached plan outright.
%
%   planRequest fields: robotModel (RobotModelContext), qCurrent (RobotState's
%   current pose -- the planner's context needs it, even though the cache
%   key below deliberately does NOT: a plan is still valid regardless of
%   where qCurrent happens to be right now), headMeshWorld,
%   headMeshSafetyWorld, headCenterWorldM, rWorldHead (R_world_head),
%   basePlacementId, haircutState, regionName, targetLengthM, plannerName,
%   options (struct: LaneIndices, PlanTransits, OnProgress -- all optional,
%   same defaults as hairsim.api.planRegion), useCache (default true).
%
%   A region planner's `context` argument mixes robot-model and scene
%   fields; this function is exactly where that merge happens, so no other
%   subsystem has to know about it.
%
%   planResult fields: regionPlan, fromCache (true if the returned plan was
%   loaded from disk, not freshly computed), cacheReason (why a cache file
%   was or wasn't used -- "no cached plan" when there simply wasn't one),
%   provenance (struct: regionName, targetLengthM, plannerName,
%   basePlacementId -- the EXACT request this plan was made for, frozen at
%   this call. The App shell must display THIS, not the live UI field
%   values, in its Plan status panel -- after the user changes a dropdown
%   without re-Planning, provenance.plannerName (say) still correctly
%   reads the OLD preset the displayed plan was actually made with, while
%   the "current settings" shown alongside it reads the new dropdown
%   value. Comparing the two is exactly how "Outdated" is detected -- never
%   by re-deriving provenance from whatever the UI happens to show now).

arguments
    planningState (1, 1) struct
    planRequest (1, 1) struct
end

if ~isfield(planRequest, 'useCache')
    planRequest.useCache = true;
end
if ~isfield(planRequest, 'options')
    planRequest.options = struct();
end

method = planRequest.plannerName;
if planRequest.basePlacementId ~= "fixed"
    method = method + "_" + planRequest.basePlacementId;
end
cacheContext = struct( ...
    'baseId', planRequest.basePlacementId, ...
    'qHome', planRequest.robotModel.qHome, ...
    'headCenterWorldM', planRequest.headCenterWorldM, ...
    'R_world_head', planRequest.rWorldHead);

provenance = struct( ...
    'regionName', planRequest.regionName, ...
    'targetLengthM', planRequest.targetLengthM, ...
    'plannerName', planRequest.plannerName, ...
    'basePlacementId', planRequest.basePlacementId);
planResult = struct('regionPlan', [], 'fromCache', false, 'cacheReason', "", 'provenance', provenance);
if planRequest.useCache
    [cachedPlan, cacheReason] = hairsim.motion.regionPlanCache('load', planningState.configDir, ...
        method, planRequest.regionName, planRequest.targetLengthM, cacheContext);
    planResult.cacheReason = cacheReason;
    if ~isempty(cachedPlan)
        planResult.regionPlan = cachedPlan;
        planResult.fromCache = true;
        planningState.lastPlan = planResult.regionPlan;
        planningState.lastPlanProvenance = provenance;
        return;
    end
end

context = struct( ...
    'robot', planRequest.robotModel.robot, ...
    'endEffector', planRequest.robotModel.endEffector, ...
    'toolMount', planRequest.robotModel.toolMount, ...
    'headCollision', {planRequest.robotModel.headCollision}, ...
    'qHome', planRequest.robotModel.qHome, ...
    'rootDir', planRequest.robotModel.rootDir, ...
    'qCurrent', planRequest.qCurrent, ...
    'headMeshWorld', planRequest.headMeshWorld, ...
    'headMeshSafetyWorld', planRequest.headMeshSafetyWorld, ...
    'headCenterWorldM', planRequest.headCenterWorldM, ...
    'R_world_head', planRequest.rWorldHead);
% the post-processor's rail station search needs these two fields; only
% present on robotModel when hairsim.app.robot.switchBase last placed the
% robot on the rail.
if isfield(planRequest.robotModel, 'railContextBuilder')
    context.railContextBuilder = planRequest.robotModel.railContextBuilder;
    context.railThetaCurrentRad = planRequest.robotModel.railThetaCurrentRad;
end

optionArgs = {};
for fieldName = ["OnProgress", "LaneIndices", "PlanTransits"]
    if isfield(planRequest.options, fieldName)
        optionArgs = [optionArgs, {char(fieldName), planRequest.options.(fieldName)}]; %#ok<AGROW>
    end
end
planResult.regionPlan = hairsim.api.planRegion(planRequest.haircutState, planRequest.regionName, ...
    planRequest.targetLengthM, context, planRequest.plannerName, optionArgs{:});

if planRequest.useCache
    try
        hairsim.motion.regionPlanCache('save', planningState.configDir, method, ...
            planResult.regionPlan, cacheContext);
    catch cacheError
        warning("hairsim:PlanCacheSaveFailed", "Could not cache plan: %s", cacheError.message);
    end
end
planningState.lastPlan = planResult.regionPlan;
planningState.lastPlanProvenance = provenance;
end
