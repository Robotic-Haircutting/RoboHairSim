function query = createHeadDistanceQuery(headMeshWorld)
%CREATEHEADDISTANCEQUERY Precomputed exact head-mesh data for
%   hairsim.tool.signedDistanceToHead. Toolbox-free (no KD tree): faces are
%   pre-culled per call by centroid distance minus face radius, then the
%   survivors get an exact point-triangle distance.
%
%   headMeshWorld: struct with faces (F,3) and vertices (V,3) in metres
%   (scene.headScene.headMeshWorld). Its face winding is outward (signed
%   volume > 0 on the shipped HeadScan.stl), so the sign of a distance is
%   taken from the closest face's normal: positive = outside the head.

arguments
    headMeshWorld (1, 1) struct
end

V = double(headMeshWorld.vertices);
F = double(headMeshWorld.faces);
A = V(F(:, 1), :);
B = V(F(:, 2), :);
C = V(F(:, 3), :);
normals = cross(B - A, C - A, 2);
areas = 0.5 * vecnorm(normals, 2, 2);
normals = normals ./ max(vecnorm(normals, 2, 2), eps);
centroids = (A + B + C) / 3;
radii = max([vecnorm(A - centroids, 2, 2), vecnorm(B - centroids, 2, 2), vecnorm(C - centroids, 2, 2)], [], 2);

query = struct();
query.A = A;
query.B = B;
query.C = C;
query.normals = normals;
query.areas = areas;
query.centroids = centroids;
query.radii = radii;
query.maxFaceRadiusM = max(radii);
end
