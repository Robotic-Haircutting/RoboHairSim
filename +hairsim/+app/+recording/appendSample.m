function recordingState = appendSample(recordingState, sample, T_world_ee_actual)
%APPENDSAMPLE Append one RecordingSample to both logs this subsystem owns
%   (the per-sample pose/cut log and the clipper-tip trajectory recorder).
%   Recording never builds `sample` itself -- per the plan's contract, the
%   App shell assembles it from a PoseCommand + RobotModelContext/RobotState
%   (FK) + a HairStepResult, so Recording never needs to know whether a
%   sample came from Teleoperation, Auto execution, or anywhere else.
%
%   sample fields (the existing contract
%   hairsim.control.appendRobotHaircutSample already expects): time, mode,
%   status, solved, cutFlag, governorActive, toolTipErrorM, cutInfo, q,
%   targetTipWorldM, desiredTipWorldM, safeTipWorldM, actualTipWorldM.
%
%   T_world_ee_actual: the 4x4 end-effector transform for this sample
%   (needed only by the clipper trajectory recorder, which derives tip
%   position/orientation from it independently of `sample`).

arguments
    recordingState (1, 1) struct
    sample (1, 1) struct
    T_world_ee_actual (4, 4) double {mustBeFinite}
end

recordingState.trajectoryRecorder = hairsim.control.appendClipperTrajectorySample( ...
    recordingState.trajectoryRecorder, T_world_ee_actual, sample.cutFlag);
[recordingState.sampleLog, recordingState.cutEvents] = hairsim.control.appendRobotHaircutSample( ...
    recordingState.sampleLog, recordingState.cutEvents, sample);
end
