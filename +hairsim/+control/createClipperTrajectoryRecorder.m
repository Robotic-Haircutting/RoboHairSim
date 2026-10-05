function recorder = createClipperTrajectoryRecorder(startTime, T_ee_tool)
%CREATECLIPPERTRAJECTORYRECORDER Create mutable clipper trajectory sample state.

recorder = struct();
recorder.startTime = startTime;
recorder.T_ee_tool = T_ee_tool;
recorder.samplesTime = [];
recorder.samplesPosition = zeros(0, 3);
recorder.samplesEuler = zeros(0, 3);
recorder.samplesCutFlag = [];
end
