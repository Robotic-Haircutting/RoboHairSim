function [recordingState, result] = finalizeSession(recordingState, ...
        robotModelContext, scenePaths, headMeshSafetyWorld, initialActiveNodeCounts, ...
        finalHaircutState, autoPlanHistory, config)
%FINALIZESESSION Write every output file (CSVs, metrics.json, summary.txt)
%   and return the same `result` struct (incl. result.metrics)
%   run_robot_haircut.m's endHaircutSession already produced. Thin wrapper
%   around hairsim.control.finalizeRobotHaircutRun -- that function needs
%   data from several subsystems (Recording's own logs, the robot model,
%   the scene's paths/head-safety-mesh, Hair/Sim's node counts, Auto
%   Planning's history, and the session config), so the App shell passes
%   all of it explicitly here rather than this function reaching into any
%   of those subsystems itself.
%
%   result.metrics is what hairsim.app.view.renderMetricsText's "saved"
%   mode formats into text; this function never touches a widget.

arguments
    recordingState (1, 1) struct
    robotModelContext (1, 1) struct
    scenePaths (1, 1) struct
    headMeshSafetyWorld (1, 1) struct
    initialActiveNodeCounts
    finalHaircutState (1, 1) struct
    autoPlanHistory (1, :) cell
    config (1, 1) struct
end

record = struct();
record.runPaths = recordingState.runPaths;
record.trajectoryRecorder = recordingState.trajectoryRecorder;
record.sampleLog = recordingState.sampleLog;
record.cutEvents = recordingState.cutEvents;
record.paths = scenePaths;
record.robot = robotModelContext.robot;
record.endEffector = robotModelContext.endEffector;
record.toolMount = robotModelContext.toolMount;
record.headMeshSafetyWorld = headMeshSafetyWorld;
record.initialActiveNodeCounts = initialActiveNodeCounts;
record.finalHaircutState = finalHaircutState;
record.autoPlanHistory = autoPlanHistory;
record.config = config;

result = hairsim.control.finalizeRobotHaircutRun(record);
recordingState.isActive = false;
recordingState.isFinalized = true;
end
