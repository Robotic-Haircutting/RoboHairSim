function executePlan(regionPlan, robotModelContext, options)
%EXECUTEPLAN Headless replay of a finished regionPlan -- no GUI required.
%   Promotes playSavedRegionPlan.m's replay logic onto hairsim.execution
%   (the same core engine the interactive Auto session uses), so the GUI and this headless entry point share one
%   implementation of "walk the plan's stroke/transit queue" -- see
%   docs/EXTENDING.md's dependency-direction note: this file depends on
%   hairsim.execution/hairsim.motion/hairsim.hair/hairsim.robot directly,
%   never on hairsim.app (a headless caller must never need a GUI
%   dependency to exist).
%
%   executePlan(regionPlan, robotModelContext)
%   executePlan(regionPlan, robotModelContext, 'ReplayHair', true, ...
%       'HeadScene', headScene, 'HairLayer', hairLayer)
%
%   robotModelContext: robot, endEffector, toolMount (headCollision/qHome
%   are not needed for replay -- nothing is re-solved or re-checked here,
%   exactly as the original playSavedRegionPlan.m's own docs state).
%
%   options.ReplayHair (default false): also cut hair sample by sample via
%   hairsim.hair.updateHairLayerForClipper, driven by each segment's own
%   cutMask, with the comb-lift clipper cut model of the plan's planner and
%   length goal (the predicate the plan's coverage was computed with).
%   Requires options.HeadScene and options.HairLayer.
%   options.ShowLaneOverlay (default true): draw the static
%   hairsim.analysis.drawRegionPlanPreview overlay once before animating.
%   Requires options.HeadScene (the overlay needs nothing else, but the
%   figure setup below does).
%   options.FrameDelayS (default 0): pause between frames.
%   options.RailContextBuilder (default []): required only if regionPlan
%   contains a feasible rail-slide transit (transit.usedRailSlide) -- see
%   hairsim.robot.createRailContextBuilder. Without it, a plan requiring a
%   mid-replay station switch errors clearly rather than silently
%   replaying at the wrong station.
%   options.Figure (default []): reuse an existing figure; otherwise one
%   is created.

arguments
    regionPlan (1, 1) struct
    robotModelContext (1, 1) struct
    options.ReplayHair (1, 1) logical = false
    options.ShowLaneOverlay (1, 1) logical = true
    options.FrameDelayS (1, 1) double {mustBeNonnegative} = 0
    options.HeadScene = []
    options.HairLayer = []
    options.RailContextBuilder = []
    options.Figure = []
end

if options.ReplayHair && (isempty(options.HeadScene) || isempty(options.HairLayer))
    error("hairsim:ExecutePlanMissingHairInputs", ...
        "options.ReplayHair requires options.HeadScene and options.HairLayer.");
end

fig = options.Figure;
if isempty(fig)
    fig = figure('Name', sprintf('Replay: %s, %.1f cm', regionPlan.region, 100 * regionPlan.targetLengthM), ...
        'Color', 'w');
end
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.06, 0.08, 0.88, 0.86]);
hairsim.hideAxesToolbar(ax);
axis(ax, 'equal');
view(ax, 135, 15);

if options.ShowLaneOverlay
    hairsim.analysis.drawRegionPlanPreview(ax, regionPlan);
end
hairLayer = options.HairLayer;
if options.ReplayHair
    hairLayer.haircutState.lengthGoal = regionPlan.lengthGoal;
    hairLayer.cutting.model = hairsim.hair.cutting.createCombLiftCutModelForPlanner( ...
        regionPlan.plannerName, hairLayer.haircutState, options.HeadScene.headMeshWorld);
    hairsim.hair.render.updateHaircutLayerOverlay(ax, hairLayer.haircutState);
end
drawFrame(regionPlan.qStartAtPlan, false, "START");
drawnow;

[executionState, ~] = hairsim.execution.loadPlan(regionPlan);
[executionState, segmentInfo] = hairsim.execution.startRun(executionState);
reportSegmentStart(segmentInfo);

while executionState.isActive
    if ~ishandle(fig)
        return;
    end
    [executionState, poseCommand, info] = hairsim.execution.stepExecution(executionState, robotModelContext);
    if ~isempty(info.railStationSwitchRequest)
        if isempty(options.RailContextBuilder)
            error("hairsim:ExecutePlanMissingRailContextBuilder", ...
                "This plan requires a mid-replay rail station switch to %.1f deg; " + ...
                "pass options.RailContextBuilder (hairsim.robot.createRailContextBuilder) to replay it.", ...
                rad2deg(info.railStationSwitchRequest));
        end
        built = options.RailContextBuilder(info.railStationSwitchRequest);
        robotModelContext.robot = built.robot;
        robotModelContext.headCollision = built.headCollision;
        continue; % re-call stepExecution for this same sample with the updated context
    end
    if isempty(poseCommand)
        continue;
    end
    drawFrame(poseCommand.qCandidate, poseCommand.cutFlag, poseCommand.statusLabel);
    if options.ReplayHair
        T_world_ee = getTransform(robotModelContext.robot, poseCommand.qCandidate, char(robotModelContext.endEffector));
        headRadiusApproxM = 0.5 * max(options.HeadScene.headMeshWorld.bboxSize);
        floorZ = options.HeadScene.headMeshWorld.bboxMin(3) - 0.35 * headRadiusApproxM;
        [hairLayer, hairUpdateInfo] = hairsim.hair.updateHairLayerForClipper( ...
            hairLayer, T_world_ee, poseCommand.cutFlag, options.HeadScene.headHullWorld, floorZ);
        if hairUpdateInfo.shouldRender
            hairsim.hair.render.updateHaircutLayerOverlay(ax, hairLayer.haircutState);
        end
    end
    if options.FrameDelayS > 0
        pause(options.FrameDelayS);
    else
        drawnow limitrate;
    end
    if info.segmentJustCompleted && ~info.runComplete
        reportSegmentStart(info.nextSegmentInfo);
    end
end
fprintf('Replay done: %d of %d strokes.\n', executionState.strokesDone, regionPlan.numStrokes);

    function drawFrame(q, cutFlag, poseStatus)
        T_world_ee_actual = getTransform(robotModelContext.robot, q, char(robotModelContext.endEffector));
        hairsim.motion.drawRobotClipperScene(ax, robotModelContext.robot, q, options.HeadScene.headMeshWorld, ...
            T_world_ee_actual, robotModelContext.toolMount, cutFlag, true, 0, 'PoseStatus', string(poseStatus));
    end

    function reportSegmentStart(segInfo)
        if isempty(segInfo)
            return;
        end
        if segInfo.kind == "stroke"
            label = sprintf('stroke %d/%d (lane %d)', segInfo.strokeOrdinal, segInfo.numStrokesInRun, segInfo.laneIdx);
        else
            label = sprintf('transit (lane %d -> lane %d)', segInfo.fromLaneIdx, segInfo.toLaneIdx);
        end
        fprintf('Replaying %s: %d samples (%d CUT).\n', label, segInfo.numSamples, segInfo.numCutSamples);
    end
end
