function T_world_base = createRailBaseTransform(railAxisWorldM, thetaRad, railRadiusM)
%CREATERAILBASETRANSFORM World pose of the robot base on a circular floor rail.
%   railAxisWorldM is the point on the rail's vertical axis at BASE height
%   ([x y zBase]); the carriage sits at radius railRadiusM and angle thetaRad
%   (about world +Z) and its base +x always points radially inward to the axis:
%       T = Trans(axis) * Rz(theta) * Trans(r, 0, 0) * Rz(pi).
%   Both the base position and its yaw change with theta (a 1-DOF coupled
%   circular base pose, not a pure translation).
%   With axis = headPos + [0 0 -0.04], r = 0.60 and theta = pi and the default
%   head at [0.60 0 0.04] this is exactly eye(4) (the current fixed base).

arguments
    railAxisWorldM (1, 3) double {mustBeFinite}
    thetaRad (1, 1) double {mustBeFinite}
    railRadiusM (1, 1) double {mustBePositive}
end
T_world_base = trvec2tform(railAxisWorldM) * axang2tform([0, 0, 1, thetaRad]) ...
    * trvec2tform([railRadiusM, 0, 0]) * axang2tform([0, 0, 1, pi]);
end
