function raw = ur5e()
%UR5E Robot adapter spec for the Universal Robots UR5e.
%   Reference implementation of the robot adapter contract (see
%   hairsim.robot.registerRobotAdapter). qHomeConstant is a Home captured
%   for the default head scene: hairsim.robot.createRobotAdapter checks it
%   against the real clipper model and re-solves it by IK if it leaves the
%   clipper closer than 1 mm to the head. qHomeCeiling is a Home tuned for
%   the overhead mount and is used as given.

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
