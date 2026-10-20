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
showUncut = false;        % red dots on the roots still to be cut (U / Show uncut)
installCut();
parkCache = [];
headRadiusApproxM = 0.5 * max(scene.headScene.headMeshWorld.bboxSize);
floorZ = scene.headScene.headMeshWorld.bboxMin(3) - 0.35 * headRadiusApproxM;
isRunning = true;
idleHairTicks = 0;        % idle frames left in which the hair keeps settling
sessionClock = tic;
lastSpaceS = -Inf;        % when Space was last pressed/released (see isFromSpace)

fig = figure('Name', 'HairSim: Teleop', 'NumberTitle', 'off', 'Color', 'w', ...
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
    struct('Home', @onHome, 'TargetLengthChanged', @onTargetLengthChanged, 'RegionChanged', @onRegionChanged, ...
        'ToggleUncut', @onToggleUncut), ...
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

    function onHome(src, ~)
        if isFromSpace(src)
            return;
        end
        [qHome, parkCache] = hairsim.app.robot.clipperSafeHome(robotAppState, scene, parkCache);
        applyTick(hairsim.app.robot.poseCommandAt(robotAppState, qHome, "HOME"));
        teleopState = hairsim.app.teleop.syncToRobot(teleopState, robotAppState);
        updatePoseText();
        dropKeyboardFocus(src);
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
        refreshUncut();
    end

    function onToggleUncut(src, ~)
        if isFromSpace(src)
            set(src, 'Value', showUncut);   % a Space press flipped it: put it back
            return;
        end
        setUncut(logical(get(src, 'Value')));
        dropKeyboardFocus(src);
    end

    function setUncut(on)
        showUncut = on;
        if isfield(ui, 'teleop') && isgraphics(ui.teleop.uncutToggle)
            set(ui.teleop.uncutToggle, 'Value', on);
        end
        refreshUncut();
    end

    function refreshUncut()
        % redraw only while shown; the count in the readout is always live
        if exist('ax', 'var') && isgraphics(ax)
            if showUncut
                hairsim.app.view.drawUncutStrands(ax, hairsim.app.hair.uncutStrands(hairAppState.hairLayer.haircutState), ...
                    scene.headScene.headCenterWorldM);
            else
                hairsim.app.view.drawUncutStrands(ax, []);
            end
        end
    end

% =====================================================================
% Recording (writes the session to disk; cutting does not depend on it)

    function onToggleRecording(src, ~)
        if isFromSpace(src)
            return;
        end
        if recordingState.isActive
            [recordingState, recordingResult] = hairsim.app.recording.stopAndFinalize(recordingState, ...
                robotAppState, scene, initialActiveNodeCounts, hairAppState);
            hairsim.app.view.renderMetricsText(viewState, "saved", recordingResult.metrics, ...
                'RunId', string(recordingState.runPaths.runId));
        else
            recordingState = hairsim.app.recording.startSession(recordingState, ...
                numel(robotAppState.state.qCurrent), robotAppState.modelContext.toolMount.T_ee_tool);
            applyTick(hairsim.app.robot.poseCommandAt(robotAppState, robotAppState.state.qCurrent, "SESSION_START"));
        end
        refreshControls();
        dropKeyboardFocus(src);
    end

    function tf = isFromSpace(src)
        % Space (= cut) also presses a button that holds keyboard focus;
        % such a press comes with a Space key event, a mouse click does not
        tf = toc(sessionClock) - lastSpaceS < 0.5;
        if tf
            dropKeyboardFocus(src);
        end
    end

    function dropKeyboardFocus(src)
        % a focused button is pressed again by Space (= cut): give the keys
        % back to the figure, as after a popup change
        if isgraphics(src)
            set(src, 'Enable', 'off');
            drawnow;
            set(src, 'Enable', 'on');
        end
    end

% =====================================================================
% Keyboard -> Teleoperation -> shared tick

    function onKeyDown(~, event)
        noteSpace(event);
        if strcmpi(event.Key, 'u')
            setUncut(~showUncut);
            return;
        end
        teleopState = hairsim.app.teleop.handleKey(teleopState, event.Key, true);
    end

    function onKeyUp(~, event)
        noteSpace(event);
        teleopState = hairsim.app.teleop.handleKey(teleopState, event.Key, false);
    end

    function noteSpace(event)
        if strcmpi(event.Key, 'space')
            lastSpaceS = toc(sessionClock);
        end
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
        if showUncut && poseCommand.cutFlag
            refreshUncut();
        end
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
        uncut = hairsim.app.hair.uncutStrands(hairAppState.hairLayer.haircutState);
        set(ui.teleop.poseText, 'String', sprintf([ ...
            'Pose      %s\n', 'Region    %s\n', 'Done      %d / %d (%.1f%%), %d left\n', 'Rail      %s\n', ...
            'Height    %.1f mm (touching: %s)\n', ...
            'Feed      %.0f deg off against-flow\n', 'Roll      %.0f deg\n', 'Clearance %.1f mm'], ...
            h.status, region, uncut.numDone, uncut.numNeeds, 100 * uncut.numDone / max(1, uncut.numNeeds), ...
            numel(uncut.idx), rail, h.standoffMm, contact, h.feedAngleDeg, h.rollDeg, h.clearanceMm));
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
