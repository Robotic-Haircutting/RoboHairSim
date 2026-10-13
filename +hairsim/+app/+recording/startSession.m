function recordingState = startSession(recordingState, numJoints, toolMountTEeTool)
%STARTSESSION Reset all logs and the session clock, and mark recording
%   active. Ported from run_robot_haircut.m's startHaircutSession (minus
%   the SESSION_START sample -- append that via
%   hairsim.app.recording.appendSample once the caller has the current
%   pose, since Recording doesn't read the robot itself).
%   toolMountTEeTool is toolMount.T_ee_tool (needed to reset the clipper
%   trajectory recorder, which converts joint samples to tip position).
%   A recording that was already finalized is never overwritten: starting
%   again opens a new run folder next to it.
if recordingState.isFinalized
    runsDir = fileparts(recordingState.runPaths.runDir);
    runId = "robot_haircut_" + string(datetime('now', 'Format', "yyyyMMdd'T'HHmmss"));
    candidate = runId;
    for k = 2:99
        if ~isfolder(fullfile(runsDir, candidate)) || isempty(dir(fullfile(runsDir, candidate, '*.csv')))
            break;
        end
        candidate = runId + "_" + k;
    end
    recordingState.runPaths = hairsim.control.createRobotHaircutRunPaths(recordingState.runPaths.root, candidate);
    recordingState.isFinalized = false;
end
recordingState.startTime = tic;
recordingState.trajectoryRecorder = hairsim.control.createClipperTrajectoryRecorder( ...
    recordingState.startTime, toolMountTEeTool);
recordingState.sampleLog = hairsim.control.createRobotHaircutSampleLog(numJoints);
recordingState.cutEvents = hairsim.control.createRobotHaircutCutEventLog();
recordingState.isActive = true;
end
