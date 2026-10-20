function registerRobotAdapter(name, builderFn)
%REGISTERROBOTADAPTER Register a robot adapter builder under a name.
%   hairsim.robot.registerRobotAdapter(name, builderFn)
%
%   builderFn is a zero-argument function handle returning a raw adapter
%   spec struct -- the contract a new robot must satisfy to plug into
%   hairsim.robot.createRobotAdapter. See +hairsim/+robot/+adapters/ur5e.m
%   for the reference implementation ("Add a robot" in README.md shows
%   the registration). The returned struct has fields:
%     name              (1,1) string, e.g. "ur5e"
%     loadRobotName     (1,1) string, passed to loadrobot()
%     endEffector       (1,1) string, end-effector body name
%     weights           (1,6) double, inverseKinematics solver weights
%     qHomeConstant     (:,1) double, a pre-tuned safe Home joint
%                       configuration for this robot, or zeros(0,1) if none
%                       exists yet (createRobotAdapter then falls back to
%                       the robot's own homeConfiguration and re-solves a
%                       safe Home via IK -- always correct, just slower on
%                       first use; see createRobotAdapter.m)
%     qHomeCeiling      (:,1) double, a pre-tuned Home for the ceiling
%                       mount, or zeros(0,1) if none exists yet (same
%                       fallback behavior as qHomeConstant)
%
%   Built-in adapters ("ur5e", "gen3") are registered automatically the
%   first time hairsim.robot.createRobotAdapter is called.

arguments
    name (1, 1) string
    builderFn (1, 1) function_handle
end
robotAdapterStore("register", name, builderFn);
end
