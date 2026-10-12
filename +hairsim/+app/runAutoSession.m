function runAutoSession(options)
%RUNAUTOSESSION Interactive Auto entry: plan one scalp region with a
%   registered planner, preview / save the plan, run it on the robot,
%   record. Teleoperation is its own entry (hairsim.app.runTeleopSession).
%
%   Same subsystem calls as the full demo (hairsim.app.runFullDemoSession):
%   planning engine, execution engine, the shared per-tick body
%   hairsim.app.applyPoseTick. This function's job, every callback, is to
%   read session.phase, build a request from a subsystem's public state,
%   call one subsystem function and refresh control enablement -- no IK,
%   collision checking or hair cutting happens here.
%
%   session.phase: idle -> planning -> planReady -> executing <-> paused,
%   back to planReady when a run ends or is aborted. A change of robot
%   setup or of the Auto fields drops planReady back to idle (the plan no
%   longer matches); the plan status panel shows the plan's own frozen
%   provenance and flags it Outdated.
%
%   The planner list is every registered planner
%   (hairsim.planning.listRegionPlanners); a release build registers only
%   the camClipper family.

arguments
    options.AdapterName (1, 1) string = "ur5e"
    options.DataDir (1, 1) string = ""
    options.PlanningDataDir (1, 1) string = ""
    options.ConfigDir (1, 1) string = ""
    options.RegionName (1, 1) string = "scalp_top_left"
    options.TargetLengthM (1, 1) double {mustBeNonnegative} = 0.005
    options.PlannerName (1, 1) string = "camClipperAgainstFlow"
    options.LaneIndices (1, :) double = []    % lane planners only; camClipper plans every segment
end

sceneArgs = {};
if strlength(options.DataDir) > 0
    sceneArgs = [sceneArgs, {'DataDir', options.DataDir}];
end
if strlength(options.PlanningDataDir) > 0
    sceneArgs = [sceneArgs, {'PlanningDataDir', options.PlanningDataDir}];
end
scene = hairsim.api.loadScene(sceneArgs{:});
robotAppState = hairsim.app.robot.createRobotAppState(options.AdapterName, scene);
hairAppState = hairsim.app.hair.createHairAppState(scene);
initialActiveNodeCounts = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;

configDir = options.ConfigDir;
if strlength(configDir) == 0
    configDir = fullfile(scene.rootDir, 'config');
end
planningState = hairsim.app.planning.createAutoPlanningState(configDir);
recordingState = hairsim.app.recording.createRecordingState( ...
    scene.rootDir, numel(robotAppState.state.qCurrent), robotAppState.modelContext.toolMount);
executionState = hairsim.execution.createExecutionState();
parkCache = [];
idleHairTicks = 0;        % idle frames left in which the hair keeps settling
headRadiusApproxM = 0.5 * max(scene.headScene.headMeshWorld.bboxSize);
floorZ = scene.headScene.headMeshWorld.bboxMin(3) - 0.35 * headRadiusApproxM;

plannerNames = hairsim.planning.listRegionPlanners();
if ~any(plannerNames == options.PlannerName)
    error("hairsim:UnknownRegionPlanner", "Planner '%s' is not registered. Registered: %s", ...
        options.PlannerName, strjoin(plannerNames, ', '));
end
session = struct();
session.phase = "idle";
session.isRunning = true;
session.pendingConfig = struct('regionName', options.RegionName, ...
    'targetLengthM', options.TargetLengthM, 'plannerName', options.PlannerName);

fig = figure('Name', 'HairSimApp: Auto', 'NumberTitle', 'off', 'Color', 'w', ...
    'Units', 'normalized', 'Position', [0.05, 0.05, 0.85, 0.85], 'CloseRequestFcn', @closeSession);
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.06, 0.08, 0.70, 0.82]);
hairsim.hideAxesToolbar(ax);
viewState = hairsim.app.view.createViewState(ax, 'CutHairColor', [0.10, 0.75, 0.20]);   % cut strands in green

ui = hairsim.app.view.createSessionControls(fig, ...
    struct('RobotSetupChanged', @onRobotSetupChanged, 'ToggleRecording', @onToggleRecording), ...
    struct('CenterTitle', 'Auto', ...
        'HairModelingConfig', hairsim.hair.createControlConfigFromLayer(hairAppState.hairLayer), ...
        'HairModelingCallbacks', hairsim.app.hair.createModelingCallbacks(@onHairChanged)));
ui.auto = hairsim.app.view.createAutoPanel(ui.centerPanel, struct( ...
    'AutoSettingsChanged', @onAutoSettingsChanged, 'Plan', @onPlan, 'Preview', @onPreview, ...
    'SavePlan', @onSavePlan, 'Run', @onRun, 'Pause', @onPause, 'Resume', @onResume, 'Abort', @onAbort), ...
    struct('PlannerOptions', plannerNames, 'Planner', options.PlannerName, ...
        'Region', options.RegionName, 'TargetHairLengthM', options.TargetLengthM));

parkRobotClipperSafe();   % the adapter Home leaves the real clipper inside the head
refreshControlEnablement();
hairsim.app.view.renderSessionScene(viewState, robotAppState, hairAppState, scene);

while ishandle(fig) && session.isRunning
    drawnow limitrate;
    if ~ishandle(fig)
        break;
    end
    if session.phase == "executing"
        tickExecution();
    elseif idleHairTicks > 0 || ~isempty(hairAppState.hairLayer.haircutState.debrisList)
        % paused / finished / idle: the arm stands still, the hair does not
        hairAppState = hairsim.app.stepIdleHair(hairAppState, robotAppState, viewState, scene, floorZ);
        idleHairTicks = max(0, idleHairTicks - 1);
    end
    pause(0.03);
end
if ishandle(fig)
    delete(fig);
end

% =====================================================================
% Robot Setup

    function onRobotSetupChanged(~, ~)
        if ~ismember(session.phase, ["idle", "planReady"])
            return;
        end
        basePlacementId = ui.basePlacementOptions(ui.robotSetupPopup.Value).id;
        robotAppState = hairsim.app.robot.switchBase(robotAppState, basePlacementId);
        parkRobotClipperSafe();   % switchBase puts the robot at the adapter Home
        if session.phase == "planReady"
            session.phase = "idle"; % a base change invalidates the current plan
        end
        updatePlanStatusText();
        hairsim.app.view.renderSessionScene(viewState, robotAppState, hairAppState, scene);
        refreshControlEnablement();
    end

    function parkRobotClipperSafe()
        setStatus("Status: computing clipper-safe Home...");
        drawnow;
        [qPark, parkCache] = hairsim.app.robot.clipperSafeHome(robotAppState, scene, parkCache);
        robotAppState.state.qCommanded = qPark;
        robotAppState.state.qCurrent = qPark;
    end

% =====================================================================
% Recording (independent of session.phase: may start/stop mid-run)

    function onToggleRecording(~, ~)
        if recordingState.isActive
            [recordingState, recordingResult] = hairsim.app.recording.stopAndFinalize(recordingState, ...
                robotAppState, scene, initialActiveNodeCounts, hairAppState);
            hairsim.app.view.renderMetricsText(viewState, "saved", recordingResult.metrics, ...
                'RunId', string(recordingState.runPaths.runId));
        elseif session.phase ~= "planning"
            recordingState = hairsim.app.recording.startSession(recordingState, ...
                numel(robotAppState.state.qCurrent), robotAppState.modelContext.toolMount.T_ee_tool);
        end
        refreshControlEnablement();
    end

% =====================================================================
% Auto fields (never plan on their own)

    function onAutoSettingsChanged(~, ~)
        if ~ismember(session.phase, ["idle", "planReady"])
            return;
        end
        session.pendingConfig.regionName = ui.auto.regionOptions(ui.auto.regionPopup.Value);
        session.pendingConfig.targetLengthM = parseLengthCm(ui.auto.targetLengthEdit.String);
        session.pendingConfig.plannerName = ui.auto.plannerOptions(ui.auto.plannerPopup.Value);
        if session.phase == "planReady"
            session.phase = "idle"; % a config change invalidates the current plan
        end
        updatePlanStatusText();
        refreshControlEnablement();
    end

% =====================================================================
% Plan is the ONLY action that produces a new plan

    function onPlan(~, ~)
        if ~ismember(session.phase, ["idle", "planReady"])
            return;
        end
        session.phase = "planning";
        refreshControlEnablement();
        drawnow;
        planRequest = struct( ...
            'robotModel', robotAppState.modelContext, ...
            'qCurrent', robotAppState.state.qCurrent, ...
            'headMeshWorld', scene.headScene.headMeshWorld, ...
            'headCenterWorldM', scene.headScene.headCenterWorldM, ...
            'rWorldHead', scene.headScene.R_world_head, ...
            'basePlacementId', robotAppState.state.basePlacementId, ...
            'haircutState', hairAppState.hairLayer.haircutState, ...
            'regionName', session.pendingConfig.regionName, ...
            'targetLengthM', session.pendingConfig.targetLengthM, ...
            'plannerName', session.pendingConfig.plannerName, ...
            'options', planOptions(), ...
            'useCache', true);
        try
            [planningState, ~] = hairsim.app.planning.planHaircut(planningState, planRequest);
            session.phase = "planReady";
        catch planError
            session.phase = "idle";
            warning("hairsim:AutoPlanFailed", "Plan failed: %s", planError.message);
        end
        updatePlanStatusText();
        refreshControlEnablement();
    end

    function onPreview(~, ~)
        if session.phase ~= "planReady" || isempty(planningState.lastPlan)
            return;
        end
        delete(findobj(ax, 'Tag', 'HairSimRegionPlan'));
        hairsim.analysis.drawRegionPlanPreview(ax, planningState.lastPlan);
    end

    function onSavePlan(~, ~)
        if session.phase ~= "planReady" || isempty(planningState.lastPlan)
            return;
        end
        regionPlan = planningState.lastPlan; %#ok<NASGU>
        defaultName = sprintf('region_plan_%s_%.1fcm.mat', ...
            planningState.lastPlanProvenance.regionName, 100 * planningState.lastPlanProvenance.targetLengthM);
        [fileName, pathName] = uiputfile(defaultName, 'Save Plan');
        if isequal(fileName, 0)
            return;
        end
        save(fullfile(pathName, fileName), 'regionPlan');
    end

% =====================================================================
% Run / Pause / Resume / Abort -- execution only, never recording

    function onRun(~, ~)
        if session.phase ~= "planReady" || isempty(planningState.lastPlan) || ~planningState.lastPlan.executable
            return;
        end
        [executionState, firstStroke] = hairsim.execution.loadPlan(planningState.lastPlan);
        hairAppState = applyCutModelForPlan(hairAppState, planningState.lastPlan, scene.headScene.headMeshWorld);
        if isCamClipperPlan(planningState.lastPlan) && isfinite(firstStroke.railThetaRad)
            % camClipper strokes carry their own rail station: put the base
            % there before the setup jump (in-plan switches are then served
            % by tickExecution's railStationSwitchRequest)
            robotAppState = hairsim.app.robot.applyRailStationForReplay(robotAppState, firstStroke.railThetaRad);
        end
        robotAppState.state.qCommanded = firstStroke.planResult.replayPositions(:, 1);
        robotAppState.state.qCurrent = firstStroke.planResult.replayPositions(:, 1);
        executionState = hairsim.execution.startRun(executionState);
        session.phase = "executing";
        hairsim.app.view.renderSessionScene(viewState, robotAppState, hairAppState, scene);
        refreshControlEnablement();
    end

    function onPause(~, ~)
        if session.phase ~= "executing"
            return;
        end
        session.phase = "paused";   % the tick loop simply stops calling stepExecution
        refreshControlEnablement();
    end

    function onResume(~, ~)
        if session.phase ~= "paused"
            return;
        end
        session.phase = "executing";
        refreshControlEnablement();
    end

    function onAbort(~, ~)
        if ~ismember(session.phase, ["executing", "paused"])
            return;
        end
        executionState = hairsim.execution.abortExecution(executionState);
        session.phase = "planReady";   % the plan itself is untouched
        refreshControlEnablement();
    end

    function tickExecution()
        [executionState, poseCommand, info] = hairsim.execution.stepExecution(executionState, robotAppState.modelContext);
        if ~isempty(info.railStationSwitchRequest)
            robotAppState = hairsim.app.robot.applyRailStationForReplay(robotAppState, info.railStationSwitchRequest);
            return;   % re-tick with the updated context
        end
        if isempty(poseCommand)
            return;
        end
        [robotAppState, hairAppState, recordingState] = hairsim.app.applyPoseTick(poseCommand, ...
            robotAppState, hairAppState, recordingState, viewState, scene, floorZ, ...
            'Record', recordingState.isActive, 'ModeLabel', "auto", 'ShowLiveMetrics', true);
        idleHairTicks = hairSettleTicks();
        if info.runComplete
            session.phase = "planReady";
            refreshControlEnablement();
        end
    end

% =====================================================================
% Hair modeling

    function onHairChanged(kind, value)
        [hairAppState, needsRedraw] = hairsim.app.hair.applyModelingChange(hairAppState, kind, value);
        if needsRedraw
            hairsim.app.view.renderSessionScene(viewState, robotAppState, hairAppState, scene);
        end
        idleHairTicks = hairSettleTicks();
    end

    function n = hairSettleTicks()
        % idle frames to keep the hair moving after activity: one full
        % physics pass over every strand, ten times
        physics = hairAppState.hairLayer.physics;
        n = 10 * ceil(numel(hairAppState.hairLayer.haircutState.hairState.activeNodeCounts) / physics.batchSize);
    end

% =====================================================================
% Display-only helpers

    function planOpts = planOptions()
        planOpts = struct('PlanTransits', true, 'OnProgress', @onPlanProgress);
        if ~isempty(options.LaneIndices)
            planOpts.LaneIndices = options.LaneIndices;
        end
    end

    function onPlanProgress(message)
        setStatus(sprintf("Status: planning... (%s)", message));
        drawnow limitrate;
    end

    function setStatus(text)
        if ishandle(ui.statusText)
            set(ui.statusText, 'String', text);
        end
    end

    function updatePlanStatusText()
        if isempty(planningState.lastPlan)
            set(ui.auto.planStatusText, 'String', 'No plan yet');
            return;
        end
        plan = planningState.lastPlan;
        provenance = planningState.lastPlanProvenance;
        isOutdated = provenance.regionName ~= session.pendingConfig.regionName || ...
            abs(provenance.targetLengthM - session.pendingConfig.targetLengthM) > 1e-9 || ...
            provenance.plannerName ~= session.pendingConfig.plannerName || ...
            provenance.basePlacementId ~= robotAppState.state.basePlacementId;
        statusWord = "Ready";
        if isOutdated
            statusWord = "Outdated -- re-plan required";
        end
        % the plan's own frozen provenance, never the live fields
        set(ui.auto.planStatusText, 'String', sprintf([ ...
            'Status       %s\n', 'Executable   %d / %d\n', 'Planner      %s\n', ...
            'Base         %s\n', 'Region       %s\n', 'Target       %.1f cm'], ...
            statusWord, plan.numExecutable, plan.numStrokes, provenance.plannerName, ...
            provenance.basePlacementId, provenance.regionName, 100 * provenance.targetLengthM));
    end

    function refreshControlEnablement()
        phase = session.phase;
        editable = ismember(phase, ["idle", "planReady"]);
        set([ui.robotSetupPopup, ui.auto.regionPopup, ui.auto.targetLengthEdit, ...
            ui.auto.plannerPopup, ui.auto.planButton], 'Enable', onOff(editable));
        hasPlan = phase == "planReady" && ~isempty(planningState.lastPlan);
        set([ui.auto.previewButton, ui.auto.saveButton], 'Enable', onOff(hasPlan));
        set(ui.auto.runButton, 'Enable', onOff(hasPlan && planningState.lastPlan.executable));
        set(ui.auto.pauseButton, 'Enable', onOff(phase == "executing"), 'Visible', onOff(phase ~= "paused"));
        set(ui.auto.resumeButton, 'Enable', onOff(phase == "paused"), 'Visible', onOff(phase == "paused"));
        set(ui.auto.abortButton, 'Enable', onOff(ismember(phase, ["executing", "paused"])));
        if recordingState.isActive
            set(ui.recordButton, 'String', 'Stop Recording', 'Enable', 'on');
        else
            set(ui.recordButton, 'String', 'Start Recording', 'Enable', onOff(phase ~= "planning"));
        end
        if isfield(ui, 'hair')
            set([ui.hair.rigiditySlider, ui.hair.lengthSlider, ui.hair.densitySlider, ...
                ui.hair.thicknessSlider], 'Enable', onOff(phase ~= "executing"));
        end
        setStatus(sprintf("Status: %s", phase));
    end

    function closeSession(~, ~)
        if ismember(session.phase, ["executing", "paused"])
            executionState = hairsim.execution.abortExecution(executionState);
        end
        if recordingState.isActive
            try
                recordingState = hairsim.app.recording.stopAndFinalize(recordingState, ...
                    robotAppState, scene, initialActiveNodeCounts, hairAppState);
            catch closeError
                warning("hairsim:CloseFinalizeFailed", "Could not finalize recording on close: %s", closeError.message);
            end
        end
        session.isRunning = false;
    end
end

function tf = isCamClipperPlan(regionPlan)
% the camClipper family = the planners hairsim.planning.camClipperTool knows
tf = false;
if ~isempty(regionPlan) && isfield(regionPlan, 'plannerName')
    [~, tf] = hairsim.planning.camClipperTool(regionPlan.plannerName);
end
end

function hairAppState = applyCutModelForPlan(hairAppState, regionPlan, headMeshWorld)
% Hair is cut by the model the plan was made for: a camClipper plan by the
% comb-lift clipper model to the plan's own length goal (the predicate its
% coverage was computed with); any other plan by the default cut (a combLift
% model left from an earlier camClipper run is removed).
if isCamClipperPlan(regionPlan)
    hairAppState.hairLayer.cutting.model = hairsim.hair.cutting.createCombLiftCutModelForPlanner( ...
        regionPlan.plannerName, hairAppState.hairLayer.haircutState, headMeshWorld);
    hairAppState.hairLayer.haircutState.lengthGoal = regionPlan.lengthGoal;
    hairAppState.hairLayer.haircutState.params.targetHairLengthM = regionPlan.targetLengthM;
    hairAppState.hairLayer.haircutState.params.targetHairLengthToleranceM = regionPlan.lengthGoal.toleranceM;
elseif isfield(hairAppState.hairLayer.cutting, 'model') && ~isempty(hairAppState.hairLayer.cutting.model) && ...
        hairAppState.hairLayer.cutting.model.type == "combLift"
    hairAppState.hairLayer.cutting.model = [];
end
end

function lengthM = parseLengthCm(text)
value = str2double(text);
if ~isfinite(value) || value < 0
    lengthM = 0;
else
    lengthM = value / 100;
end
end

function state = onOff(isOn)
if isOn
    state = 'on';
else
    state = 'off';
end
end
