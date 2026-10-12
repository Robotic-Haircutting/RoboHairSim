function handle = drawTeleopRegion(ax, headMeshWorld, teleopState)
%DRAWTELEOPREGION Highlight Teleoperation's selected region on the scalp:
%   the head-mesh faces inside it (hairsim.app.teleop.regionMembership, the
%   same test that limits the clipper), as a translucent patch lifted
%   1.5 mm off the scalp so it does not fight the head surface. Tag
%   'HairSimTeleopRegion': replaced on every call, survives the per-frame
%   scene redraw. No region selected: removes the highlight.

arguments
    ax
    headMeshWorld (1, 1) struct
    teleopState (1, 1) struct
end

tag = 'HairSimTeleopRegion';
delete(findobj(ax, 'Tag', tag));
handle = gobjects(0);
if strlength(teleopState.regionName) == 0
    return;
end
V = double(headMeshWorld.vertices);
F = double(headMeshWorld.faces);
centroids = (V(F(:, 1), :) + V(F(:, 2), :) + V(F(:, 3), :)) / 3;
inside = hairsim.app.teleop.regionMembership(teleopState, centroids);

faceNormals = cross(V(F(:, 2), :) - V(F(:, 1), :), V(F(:, 3), :) - V(F(:, 1), :), 2);
vertexNormals = zeros(size(V));
for k = 1:3
    vertexNormals = vertexNormals + accumarray([repmat(F(:, k), 3, 1), kron((1:3)', ones(size(F, 1), 1))], ...
        faceNormals(:), size(V));
end
vertexNormals = vertexNormals ./ max(vecnorm(vertexNormals, 2, 2), eps);

hold(ax, 'on');
handle = patch(ax, 'Faces', F(inside, :), 'Vertices', V + 0.0015 * vertexNormals, ...
    'FaceColor', [0.10, 0.50, 1.00], 'FaceAlpha', 0.60, 'EdgeColor', 'none', 'AmbientStrength', 0.6, ...
    'FaceLighting', 'gouraud', 'Tag', tag);
end
