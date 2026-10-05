function raw = ur5e()
%UR5E Robot adapter spec for the Universal Robots UR5e.
%   Reference implementation of the robot adapter contract (see
%   hairsim.robot.registerRobotAdapter and docs/EXTENDING.md). Values here
%   are the exact constants the original matlab_ur5e bootstrap used
%   (run_robot_haircut.m's setup block and exploreCeilingMountPose.m):
%   qHomeConstant is the captured Home that keeps the clipper sphere
%   already clear of the head for the default head scene (so the
%   clearance fix-up in hairsim.robot.createRobotAdapter is normally a
%   no-op for this robot), and qHomeCeiling is the tuned overhead-mount
%   Home from the ceiling-placement study.

raw = struct();
raw.name = "ur5e";
raw.loadRobotName = "universalUR5e";
raw.endEffector = "tool0";
raw.weights = [5, 5, 5, 0.01, 0.01, 0.01];
raw.qHomeConstant = [ ...
   -0.481066740805354; ...
   -1.473785083147127; ...
    2.003384337039897; ...
   -0.113173756729414; ...
    0.979288446316295; ...
   -0.241802443607492];
raw.qHomeCeiling = [-2.812385; -2.980599; 1.498042; -1.659036; -1.900004; -2.094395];
end
