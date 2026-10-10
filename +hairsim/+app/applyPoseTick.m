function [robotAppState, hairAppState, recordingState, tickCounted] = applyPoseTick( ...
        poseCommand, robotAppState, hairAppState, recordingState, viewState, scene, floorZ, options)
%APPLYPOSETICK Apply one PoseCommand, step hair, append a recording
%   sample, render. Shared by hairsim.app.runFullDemoSession (which also
%   appends to its replay bundle on top of this),
%   hairsim.app.replayFullDemoBundle (pure replay, no bundle to build) and
%   the interactive entries hairsim.app.runAutoSession /
%   hairsim.app.runTeleopSession. Promoted here so none of them duplicates
%   this per-tick body.
%
%   options.Record: append a recording sample (the interactive entries
%   record only while the user has recording on). options.ModeLabel: the
%   sample's mode ("auto" / "teleop"). options.ShowLiveMetrics: refresh the
%   live metrics text after recording.
arguments
    poseCommand (1, 1) struct
    robotAppState (1, 1) struct
    hairAppState (1, 1) struct
    recordingState (1, 1) struct
    viewState
    scene (1, 1) struct
    floorZ (1, 1) double
    options.Record (1, 1) logical = true
    options.ModeLabel (1, 1) string = "auto"
    options.ShowLiveMetrics (1, 1) logical = false
end
robotAppState = hairsim.app.robot.applyPoseCommand(robotAppState, poseCommand);
tickCounted = 1;

T_world_ee_actual = getTransform(robotAppState.modelContext.robot, robotAppState.state.qCurrent, ...
    char(robotAppState.modelContext.endEffector));
[hairAppState, hairStepResult] = hairsim.app.hair.stepHairSim(hairAppState, T_world_ee_actual, ...
    poseCommand.cutFlag, floorZ);

if options.Record
    sample = struct( ...
        'time', toc(recordingState.startTime), 'mode', options.ModeLabel, ...
        'status', hairsim.hair.appendLayerStatus(poseCommand.statusLabel, hairStepResult), ...
        'solved', poseCommand.solved, 'cutFlag', poseCommand.cutFlag, ...
        'governorActive', poseCommand.governorActive, 'toolTipErrorM', poseCommand.toolTipErrorM, ...
        'cutInfo', hairStepResult.cutInfo, 'q', robotAppState.state.qCurrent, ...
        'targetTipWorldM', poseCommand.targetTipWorldM, 'desiredTipWorldM', poseCommand.desiredTipWorldM, ...
        'safeTipWorldM', poseCommand.safeTipWorldM, 'actualTipWorldM', poseCommand.toolTipWorldM);
    recordingState = hairsim.app.recording.appendSample(recordingState, sample, T_world_ee_actual);
    if options.ShowLiveMetrics && ~isempty(viewState)
        hairsim.app.view.renderMetricsText(viewState, "live", ...
            hairsim.app.recording.queryMetrics(recordingState, "live"));
    end
end

if ~isempty(viewState)
    renderRequest = struct( ...
        'robot', robotAppState.modelContext.robot, 'qCurrent', robotAppState.state.qCurrent, ...
        'headMeshWorld', scene.headScene.headMeshWorld, 'T_world_ee_actual', T_world_ee_actual, ...
        'toolMount', robotAppState.modelContext.toolMount, ...
            'toolDisplayMesh', {robotAppState.modelContext.toolDisplayMesh}, ...
        'cutFlag', poseCommand.cutFlag, 'solved', poseCommand.solved, ...
        'toolTipErrorM', poseCommand.toolTipErrorM, 'poseStatus', poseCommand.statusLabel, ...
        'sceneZMaxM', 0.65);
    hairsim.app.view.renderScene(viewState, renderRequest);
    if hairStepResult.shouldRender
        hairsim.app.view.renderHairOverlay(viewState, hairAppState.hairLayer.haircutState);
    end
    drawnow limitrate;
end
end
