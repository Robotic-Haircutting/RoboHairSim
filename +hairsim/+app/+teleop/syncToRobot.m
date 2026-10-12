function teleopState = syncToRobot(teleopState, robotAppState)
%SYNCTOROBOT Re-anchor the surface state to where the clipper actually is
%   and drop queued input. The foot is the scalp point closest to the TCP,
%   the standoff its distance (capped at MaxStandoffM), the feed the TCP x
%   axis laid onto the tangent plane, roll 0. The robot does not move: the
%   first key press then turns/moves it onto the surface state, at most
%   MaxRotationPerTickRad / MaxTranslationPerTickM per tick.
%   Call on entering Teleop and after anything else moved the robot
%   (Home, base switch).

arguments
    teleopState (1, 1) struct
    robotAppState (1, 1) struct
end

mc = robotAppState.modelContext;
T_tcp = getTransform(mc.robot, robotAppState.state.qCurrent, char(mc.endEffector)) * teleopState.tool.T_ee_tcp;
p = T_tcp(1:3, 4)';
[d, foot] = hairsim.tool.signedDistanceToHead(p, teleopState.headQuery);
n = hairsim.cam.smoothedNormalAt(foot, teleopState.headQuery);
feed = T_tcp(1:3, 1)' - dot(T_tcp(1:3, 1)', n) * n;
if norm(feed) < 1e-6          % TCP x along the normal: any tangent will do
    feed = T_tcp(1:3, 2)' - dot(T_tcp(1:3, 2)', n) * n;
end
teleopState.surface = struct('footWorldM', foot, 'feedWorld', feed / norm(feed), ...
    'standoffM', min(max(d, teleopState.tool.nominalStandoffM), teleopState.maxStandoffM), 'rollRad', 0);
teleopState.pending = hairsim.app.teleop.emptyInput();
teleopState.spaceHeld = false;
teleopState.settling = false;
teleopState.hud.status = "SYNCED";
teleopState.hud.standoffMm = 1000 * d;
teleopState = hairsim.app.teleop.setRegion(teleopState, teleopState.regionName);   % where the clipper is now
end
