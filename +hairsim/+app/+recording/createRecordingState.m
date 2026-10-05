function recordingState = createRecordingState(rootDir, numJoints, toolMount, options)
%CREATERECORDINGSTATE Recording subsystem state: sample log, cut-event
%   log, clipper trajectory recorder, and the run's output paths. Pure
%   data -- no axes/widget handle anywhere in this struct (see the plan's
%   note on why metrics TEXT rendering is Visualization's job, not
%   Recording's: hairsim.app.recording.queryMetrics returns numbers,
%   hairsim.app.view.renderMetricsText turns them into a string).
%
%   recordingState fields: runPaths, sampleLog, cutEvents,
%   trajectoryRecorder, startTime, isActive, isFinalized.

arguments
    rootDir (1, 1) string
    numJoints (1, 1) double {mustBeInteger, mustBePositive}
    toolMount (1, 1) struct
    options.RunId (1, 1) string = ""
end

runId = options.RunId;
if strlength(runId) == 0
    runPaths = hairsim.control.createRobotHaircutRunPaths(rootDir);
else
    runPaths = hairsim.control.createRobotHaircutRunPaths(rootDir, runId);
end

recordingState = struct();
recordingState.runPaths = runPaths;
recordingState.startTime = tic;
recordingState.trajectoryRecorder = hairsim.control.createClipperTrajectoryRecorder( ...
    recordingState.startTime, toolMount.T_ee_tool);
recordingState.sampleLog = hairsim.control.createRobotHaircutSampleLog(numJoints);
recordingState.cutEvents = hairsim.control.createRobotHaircutCutEventLog();
recordingState.isActive = false;
recordingState.isFinalized = false;
end
