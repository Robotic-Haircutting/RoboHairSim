function poseCommand = poseCommandAt(robotAppState, q, statusLabel)
%POSECOMMANDAT An accepted PoseCommand that puts the robot at q with no
%   cut: the Teleop Home button, and the SESSION_START sample a recording
%   begins with (so a recording always holds the pose it started from).

arguments
    robotAppState (1, 1) struct
    q (:, 1) double
    statusLabel (1, 1) string
end

mc = robotAppState.modelContext;
tip = hairsim.getMountedClipperPoint(getTransform(mc.robot, q, char(mc.endEffector)), mc.toolMount.T_ee_tool);
poseCommand = struct('qCandidate', q, 'solved', true, 'toolTipWorldM', tip, ...
    'statusLabel', statusLabel, 'toolTipErrorM', 0, 'targetTipWorldM', tip, ...
    'desiredTipWorldM', tip, 'safeTipWorldM', tip, 'governorActive', false, 'cutFlag', false);
end
