function R = axisAngleRotation(axis, angleRad)
%AXISANGLEROTATION Rodrigues' rotation formula.
%   Shared by hairsim.createTiltedRotation and hairsim.interpolateRotation.

c = cos(angleRad);
s = sin(angleRad);
k = axis(:);
K = [
    0,     -k(3),  k(2)
    k(3),   0,    -k(1)
   -k(2),   k(1),  0
    ];
R = eye(3) * c + s * K + (1 - c) * (k * k');
end
