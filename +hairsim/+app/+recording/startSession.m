function recordingState = startSession(recordingState, numJoints, toolMountTEeTool)
%STARTSESSION Reset all logs and the session clock, and mark recording
%   active. Ported from run_robot_haircut.m's startHaircutSession (minus
%   the SESSION_START sample -- append that via
%   hairsim.app.recording.appendSample once the caller has the current
%   pose, since Recording doesn't read the robot itself).
%   toolMountTEeTool is toolMount.T_ee_tool (needed to reset the clipper
%   trajectory recorder, which converts joint samples to tip position).
recordingState.startTime = tic;
recordingState.trajectoryRecorder = hairsim.control.createClipperTrajectoryRecorder( ...
    recordingState.startTime, toolMountTEeTool);
recordingState.sampleLog = hairsim.control.createRobotHaircutSampleLog(numJoints);
recordingState.cutEvents = hairsim.control.createRobotHaircutCutEventLog();
recordingState.isActive = true;
end
