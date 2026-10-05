function R_tilted = createTiltedRotation(R, tiltAngleRad, tiltDirectionRad)
%CREATETILTEDROTATION Tilt a rotation's local +Z axis away from its exact direction.
%   R is a 3x3 rotation matrix with columns [x, y, z]. The tool axis z is
%   rotated by tiltAngleRad about an axis lying in the local x-y plane at
%   azimuth tiltDirectionRad (0 = along +x, increasing toward +y). The whole
%   frame is rotated rigidly by this axis-angle rotation, so the result is
%   automatically orthonormal -- no separate Gram-Schmidt re-orthogonalization
%   is needed.

if nargin < 3
    tiltDirectionRad = 0;
end

tiltAxis = cos(tiltDirectionRad) * R(:, 1) + sin(tiltDirectionRad) * R(:, 2);
tiltAxis = tiltAxis / norm(tiltAxis);

R_tilted = hairsim.axisAngleRotation(tiltAxis, tiltAngleRad) * R;
end
