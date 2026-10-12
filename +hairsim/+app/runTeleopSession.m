function runTeleopSession(options)
%RUNTELEOPSESSION Interactive clipper Teleoperation: drive the clipper over
%   the scalp with the keyboard and cut by pushing it forward with Space
%   held. Auto is its own entry (hairsim.app.runAutoSession).
%
%   Same clipper as Auto: the camClipper tool, pose builder and safety
%   gates (hairsim.app.teleop.stepTeleop) and the comb-lift cut model of
%   options.PlannerName at the Target length field
%   (hairsim.app.hair.installClipperCut) -- so a hand-guided pass cuts
%   exactly what the same pass would cut in Auto. Teleoperation only
%   PROPOSES a PoseCommand; it is applied through the shared per-tick body
%   hairsim.app.applyPoseTick. Cutting does not need recording; Record
%   only writes the session to disk.
%
%   Region: confine the session to one scalp region ("" = whole head). The
%   region is highlighted on the scalp, the clipper may not be driven out
%   of it (hairsim.app.teleop.setRegion) and only its strands are cut
%   (hairsim.app.hair.installClipperCut 'Region'). Also a popup in the
%   panel.
%
%   Keys are read from the figure, so click the 3D view first; after a
%   Robot Setup or Region change the popup drops keyboard focus by itself.

arguments
    options.AdapterName (1, 1) string = "ur5e"
    options.DataDir (1, 1) string = ""
    options.PlanningDataDir (1, 1) string = ""
    options.TargetLengthM (1, 1) double {mustBeNonnegative} = 0.005
    options.PlannerName (1, 1) string = "camClipperAgainstFlow"
    options.RegionName (1, 1) string = ""
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
recordingState = hairsim.app.recording.createRecordingState( ...
    scene.rootDir, numel(robotAppState.state.qCurrent), robotAppState.modelContext.toolMount);
teleopState = hairsim.app.teleop.createTeleopState(scene, 'PlannerName', options.PlannerName);
targetLengthM = options.TargetLengthM;
regionName = options.RegionName;
installCut();
parkCache = [];
headRadiusApproxM = 0.5 * max(scene.headScene.headMeshWorld.bboxSize);
floorZ = scene.headScene.headMeshWorld.bboxMin(3) - 0.35 * headRadiusApproxM;
isRunning = true;
idleHairTicks = 0;        % idle frames left in which the hair keeps settling

fig = figure('Name', 'HairSimApp: Teleop', 'NumberTitle', 'off', 'Color', 'w', ...
    'Units', 'normalized', 'Position', [0.05, 0.05, 0.85, 0.85], 'CloseRequestFcn', @closeSession, ...
    'WindowKeyPressFcn', @onKeyDown, 'WindowKeyReleaseFcn', @onKeyUp);
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.06, 0.08, 0.70, 0.82]);
hairsim.hideAxesToolbar(ax);
viewState = hairsim.app.view.createViewState(ax, 'CutHairColor', [0.10, 0.75, 0.20]);   % cut strands in green

ui = hairsim.app.view.createSessionControls(fig, ...
    struct('RobotSetupChanged', @onRobotSetupChanged, 'ToggleRecording', @onToggleRecording), ...
    struct('CenterTitle', 'Teleop', ...
        'HairModelingConfig', hairsim.hair.createControlConfigFromLayer(hairAppState.hairLayer), ...
        'HairModelingCallbacks', hairsim.app.hair.createModelingCallbacks(@onHairChanged)));
ui.teleop = hairsim.app.view.createTeleopPanel(ui.centerPanel, ...
    struct('Home', @onHome, 'TargetLengthChanged', @onTargetLengthChanged, 'RegionChanged', @onRegionChanged), ...
    struct('TargetHairLengthM', targetLengthM, 'Region', regionName, ...
        'RegionOptions', unique(hairsim.hair.getStrandRegions(hairAppState.hairLayer.haircutState))'));

parkRobotClipperSafe();   % the adapter Home leaves the real clipper inside the head
teleopState = hairsim.app.teleop.syncToRobot(teleopState, robotAppState);
refreshControls();
updatePoseText();
hairsim.app.view.renderSessionScene(viewState, robotAppState, hairAppState, scene);
hairsim.app.view.drawTeleopRegion(ax, scene.headScene.headMeshWorld, teleopState);

while ishandle(fig) && isRunning
    drawnow limitrate;
    if ~ishandle(fig)
        break;
    end
    tickTeleop();
    pause(0.03);
end
if ishandle(fig)
    delete(fig);
end

% =====================================================================
% Robot Setup / Home

    function onRobotSetupChanged(src, ~)
        basePlacementId = ui.basePlacementOptions(ui.robotSetupPopup.Value).id;
        robotAppState = hairsim.app.robot.switchBase(robotAppState, basePlacementId);
        parkRobotClipperSafe();   % switchBase puts the robot at the adapter Home
        teleopState = hairsim.app.teleop.syncToRobot(teleopState, robotAppState);
        hairsim.app.view.renderSessionScene(viewState, robotAppState, hairAppState, scene);
        % drop keyboard focus from the popup so arrow keys drive the robot
        % instead of changing the selection
        set(src, 'Enable', 'off');
        drawnow;
        refreshControls();
        updatePoseText();
    end

    function onHome(~, ~)
        [qHome, parkCache] = hairsim.app.robot.clipperSafeHome(robotAppState, scene, parkCache);
        T_world_ee = getTransform(robotAppState.modelContext.robot, qHome, char(robotAppState.modelContext.endEffector));
        homeTipWorldM = hairsim.getMountedClipperPoint(T_world_ee, robotAppState.modelContext.toolMount.T_ee_tool);
        poseCommand = struct('qCandidate', qHome, 'solved', true, 'toolTipWorldM', homeTipWorldM, ...
            'statusLabel', "HOME", 'toolTipErrorM', 0, 'targetTipWorldM', homeTipWorldM, ...
            'desiredTipWorldM', homeTipWorldM, 'safeTipWorldM', homeTipWorldM, ...
            'governorActive', false, 'cutFlag', false);
        applyTick(poseCommand);
        teleopState = hairsim.app.teleop.syncToRobot(teleopState, robotAppState);
        updatePoseText();
    end

    function parkRobotClipperSafe()
        setStatus("Status: computing clipper-safe Home...");
        drawnow;
        [qPark, parkCache] = hairsim.app.robot.clipperSafeHome(robotAppState, scene, parkCache);
        robotAppState.state.qCommanded = qPark;
        robotAppState.state.qCurrent = qPark;
    end

% =====================================================================
% Region / target length / cut model

    function onRegionChanged(src, ~)
        regionName = ui.teleop.regionOptions(src.Value);
        installCut();
        hairsim.app.view.drawTeleopRegion(ax, scene.headScene.headMeshWorld, teleopState);
        updatePoseText();
        set(src, 'Enable', 'off');   % drop keyboard focus, as after a Robot Setup change
        drawnow;
        refreshControls();
    end

    function onTargetLengthChanged(src, ~)
        value = str2double(src.String);
        if isfinite(value) && value >= 0
            targetLengthM = value / 100;
            installCut();
        end
        set(src, 'String', sprintf('%.1f', 100 * targetLengthM));
    end

    function installCut()
        [hairAppState, flowDirs] = hairsim.app.hair.installClipperCut(hairAppState, ...
            scene.headScene.headMeshWorld, options.PlannerName, targetLengthM, 'Region', regionName);
        teleopState = hairsim.app.teleop.setHair(teleopState, hairAppState.hairLayer.haircutState, flowDirs);
        teleopState = hairsim.app.teleop.setRegion(teleopState, regionName);
    end

% =====================================================================
% Recording (writes the session to disk; cutting does not depend on it)

    function onToggleRecording(~, ~)
        if recordingState.isActive
            [recordingState, recordingResult] = hairsim.app.recording.stopAndFinalize(recordingState, ...
                robotAppState, scene, initialActiveNodeCounts, hairAppState);
            hairsim.app.view.renderMetricsText(viewState, "saved", recordingResult.metrics, ...
                'RunId', string(recordingState.runPaths.runId));
        else
            recordingState = hairsim.app.recording.startSession(recordingState, ...
                numel(robotAppState.state.qCurrent), robotAppState.modelContext.toolMount.T_ee_tool);
        end
        refreshControls();
    end

% =====================================================================
% Keyboard -> Teleoperation -> shared tick

    function onKeyDown(~, event)
        teleopState = hairsim.app.teleop.handleKey(teleopState, event.Key, true);
    end

    function onKeyUp(~, event)
        teleopState = hairsim.app.teleop.handleKey(teleopState, event.Key, false);
    end

    function tickTeleop()
        [teleopState, poseCommand] = hairsim.app.teleop.stepTeleop(teleopState, robotAppState, true);
        if ~isempty(poseCommand)
            applyTick(poseCommand);
            updatePoseText();
        elseif idleHairTicks > 0 || ~isempty(hairAppState.hairLayer.haircutState.debrisList)
            % the arm stands still; the hair does not
            hairAppState = hairsim.app.stepIdleHair(hairAppState, robotAppState, viewState, scene, floorZ);
            idleHairTicks = max(0, idleHairTicks - 1);
        end
    end

    function applyTick(poseCommand)
        [robotAppState, hairAppState, recordingState] = hairsim.app.applyPoseTick(poseCommand, ...
            robotAppState, hairAppState, recordingState, viewState, scene, floorZ, ...
            'Record', recordingState.isActive, 'ModeLabel', "teleop", 'ShowLiveMetrics', true);
        idleHairTicks = hairSettleTicks();
    end

% =====================================================================
% Hair modeling / display

    function onHairChanged(kind, value)
        [hairAppState, needsRedraw] = hairsim.app.hair.applyModelingChange(hairAppState, kind, value);
        if needsRedraw
            installCut();   % rebuilt strands: new goal, new flow, new region roots
            hairsim.app.view.renderSessionScene(viewState, robotAppState, hairAppState, scene);
            hairsim.app.view.drawTeleopRegion(ax, scene.headScene.headMeshWorld, teleopState);
        end
        idleHairTicks = hairSettleTicks();
    end

    function updatePoseText()
        h = teleopState.hud;
        contact = "no";
        if h.inContact
            contact = "yes";
        end
        region = "whole head";
        if strlength(teleopState.regionName) > 0
            region = teleopState.regionName + " (inside)";
            if ~h.inRegion
                region = sprintf("%s (outside, %.0f mm)", teleopState.regionName, h.regionDistanceMm);
            end
        end
        rail = "-";
        if isfinite(robotAppState.state.railThetaRad)
            rail = sprintf("%.0f deg", rad2deg(robotAppState.state.railThetaRad));
        end
        set(ui.teleop.poseText, 'String', sprintf([ ...
            'Pose      %s\n', 'Region    %s\n', 'Rail      %s\n', 'Height    %.1f mm (touching: %s)\n', ...
            'Feed      %.0f deg off against-flow\n', 'Roll      %.0f deg\n', 'Clearance %.1f mm'], ...
            h.status, region, rail, h.standoffMm, contact, h.feedAngleDeg, h.rollDeg, h.clearanceMm));
    end

    function setStatus(text)
        if ishandle(ui.statusText)
            set(ui.statusText, 'String', text);
        end
    end

    function refreshControls()
        set([ui.robotSetupPopup, ui.teleop.regionPopup, ui.teleop.homeButton, ui.recordButton], 'Enable', 'on');
        if recordingState.isActive
            set(ui.recordButton, 'String', 'Stop Recording');
            setStatus("Status: teleop (recording)");
        else
            set(ui.recordButton, 'String', 'Start Recording');
            setStatus("Status: teleop");
        end
    end

    function n = hairSettleTicks()
        % idle frames to keep the hair moving after activity: one full
        % physics pass over every strand, ten times
        physics = hairAppState.hairLayer.physics;
        n = 10 * ceil(numel(hairAppState.hairLayer.haircutState.hairState.activeNodeCounts) / physics.batchSize);
    end

    function closeSession(~, ~)
        if recordingState.isActive
            try
                recordingState = hairsim.app.recording.stopAndFinalize(recordingState, ...
                    robotAppState, scene, initialActiveNodeCounts, hairAppState);
            catch closeError
                warning("hairsim:CloseFinalizeFailed", "Could not finalize recording on close: %s", closeError.message);
            end
        end
        isRunning = false;
    end
end
