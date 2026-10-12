function appState = applyPoseCommand(appState, poseCommand)
%APPLYPOSECOMMAND The ONLY function allowed to change RobotState's
%   qCommanded/qCurrent. Teleoperation and Execution never mutate robot
%   state themselves -- they each tick propose a PoseCommand (qCandidate,
%   solved, plus the display/recording fields: toolTipWorldM, statusLabel,
%   toolTipErrorM, targetTipWorldM, desiredTipWorldM, safeTipWorldM,
%   governorActive, cutFlag) and the App shell calls this to apply it.
%
%   qCommanded is set to poseCommand.qCandidate unconditionally (it is the
%   record of what was last asked for, accepted or not). qCurrent is set to
%   it only when poseCommand.solved is true -- a rejected candidate never
%   moves the robot. This is the simulation backend's policy (commit
%   immediately on acceptance, no actuation gap); a future hardware backend
%   would instead leave qCurrent alone here and update it only once a real
%   controller reports arrival, without changing this function's contract.
%
%   Optional poseCommand.railThetaRad: an accepted command that also moves
%   the base along the rail (Teleop's rail slide). The robot model is put at
%   that station first (hairsim.app.robot.applyRailStationForReplay); absent
%   or equal to the current station = no change.

arguments
    appState (1, 1) struct
    poseCommand (1, 1) struct
end

if poseCommand.solved && isfield(poseCommand, 'railThetaRad') && isfinite(poseCommand.railThetaRad) && ...
        ~(abs(poseCommand.railThetaRad - appState.state.railThetaRad) <= 1e-12)
    appState = hairsim.app.robot.applyRailStationForReplay(appState, poseCommand.railThetaRad);
end
appState.state.qCommanded = poseCommand.qCandidate(:);
if poseCommand.solved
    appState.state.qCurrent = poseCommand.qCandidate(:);
end
end
