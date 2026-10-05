function raw = gen3()
%GEN3 Robot adapter spec for the Kinova Gen3 (7-DOF).
%   Ported by reading matlab_gen3's bootstrap as a read-only reference
%   (matlab_gen3 itself is not modified -- it is a separate, still-active
%   research sandbox). Unlike the UR5e adapter, no Home configuration has
%   been tuned for this robot yet: matlab_gen3 itself only ever reused the
%   UR5e-tuned 6-vector and relied on a joint-count mismatch check (6 vs 7)
%   to discard it and fall back to the robot's own homeConfiguration. That
%   fallback is now a generic, built-in behavior of
%   hairsim.robot.createRobotAdapter (triggered whenever qHomeConstant/
%   qHomeCeiling is empty or the wrong size), so this adapter states its
%   "not tuned yet" status directly instead of indirectly relying on a
%   size mismatch against a different robot's constant.
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
