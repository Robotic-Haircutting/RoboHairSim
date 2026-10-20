function raw = gen3()
%GEN3 Robot adapter spec for the Kinova Gen3 (7-DOF).
%   Unlike the UR5e adapter, no Home configuration has been tuned for this
%   robot: qHomeConstant is empty, so hairsim.robot.createRobotAdapter
%   solves a clipper-safe Home by IK on first use; qHomeCeiling is empty
%   too, so the ceiling mount starts from the robot's own
%   homeConfiguration.
%
%   NOT YET VALIDATED against the frozen M3/transit benchmark numbers --
%   those were only ever run against the "ur5e" adapter. Treat this
%   adapter as proof that the registry can swap robots, not as a
%   reproduction of any published result.

raw = struct();
raw.name = "gen3";
raw.loadRobotName = "kinovaGen3";
raw.endEffector = "EndEffector_Link";
raw.weights = [5, 5, 5, 0.01, 0.01, 0.01];
raw.qHomeConstant = zeros(0, 1);
raw.qHomeCeiling = zeros(0, 1);
end
