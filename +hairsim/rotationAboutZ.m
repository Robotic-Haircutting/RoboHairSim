function R = rotationAboutZ(angleRadians)
%ROTATIONABOUTZ Return a right-handed 3D rotation about the world Z axis.

c = cos(angleRadians);
s = sin(angleRadians);
R = [ ...
    c, -s, 0; ...
    s,  c, 0; ...
    0,  0, 1];
end
