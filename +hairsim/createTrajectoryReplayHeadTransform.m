function [T_world_head, headScaleFactor] = createTrajectoryReplayHeadTransform()
%CREATETRAJECTORYREPLAYHEADTRANSFORM Return the default HeadScan placement.

headScaleFactor = 1.1;
headPositionWorldM = [0.60, 0.00, 0.04];
R_world_head = hairsim.rotationAboutZ(-pi / 2);

T_world_head = eye(4);
T_world_head(1:3, 1:3) = R_world_head;
T_world_head(1:3, 4) = headPositionWorldM(:);
end
