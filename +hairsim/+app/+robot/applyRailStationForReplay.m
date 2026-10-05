function appState = applyRailStationForReplay(appState, railThetaRad)
%APPLYRAILSTATIONFORREPLAY Swap ONLY robot/headCollision to the rail
%   station railThetaRad, for replaying a rail-slide plan segment that was
%   planned/validated at a DIFFERENT station than the one currently
%   selected. Ported from run_robot_haircut.m's setRobotStationForReplay.
%
%   Unlike hairsim.app.robot.switchBase, this never touches
%   qHome/qCurrent/qCommanded/basePlacementId -- a replay is already
%   driving qCurrent sample by sample from the saved plan; this only makes
%   the robot model agree with what the planner actually validated at this
%   point in the replay. appState.state.railThetaRad IS updated (it
%   records which station the robot model currently reflects).
arguments
    appState (1, 1) struct
    railThetaRad (1, 1) double {mustBeFinite}
end

builder = hairsim.robot.createRailContextBuilder(appState.adapter, appState.headScene, ...
    'RadiusM', appState.railRadiusM, 'BaseOffsetM', appState.railBaseOffsetM);
built = builder(railThetaRad);
appState.modelContext.robot = built.robot;
appState.modelContext.headCollision = built.headCollision;
appState.state.railThetaRad = railThetaRad;
end
