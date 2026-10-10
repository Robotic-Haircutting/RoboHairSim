function cutModel = createGuardBladeCutModel(headMeshWorld, toolMount, guardHeightM, options)
%CREATEGUARDBLADECUTMODEL Cut model for hairsim.hair.cutting.applyGuardBladeCutToGuideHair.
%   Flat blade of the clipper model (toolMount.tool): blade width =
%   2 * tool.bladeHalfWidthM, blade plane at the TCP (tool.tcpOffsetM along
%   EE z, the teeth face).
%   guardHeightM = guard-comb height = target contour offset from the scalp.
%
%   Scalp height of a point = (point - nearest scalp vertex) . outward
%   vertex normal; normals are flipped to point away from the head
%   centroid (the head is star-shaped about it).

arguments
    headMeshWorld (1, 1) struct
    toolMount (1, 1) struct
    guardHeightM (1, 1) double {mustBeNonnegative}
    options.BladeHalfDepthM (1, 1) double {mustBePositive} = 0.005
    options.BladeHeightM (1, 1) double {mustBePositive} = 2 * toolMount.tool.bladeHalfWidthM
end

vertices = double(headMeshWorld.vertices);
scalpTri = triangulation(double(headMeshWorld.faces), vertices);
normals = vertexNormal(scalpTri);
centroid = mean(vertices, 1);
inward = sum((vertices - centroid) .* normals, 2) < 0;
normals(inward, :) = -normals(inward, :);

cutModel = struct( ...
    'type', "guardBlade", ...
    'guardHeightM', guardHeightM, ...
    'bladeHalfWidthM', toolMount.tool.bladeHalfWidthM, ...
    'bladeHalfDepthM', options.BladeHalfDepthM, ...
    'bladeHeightM', options.BladeHeightM, ...
    'bladeTipZM', toolMount.tool.tcpOffsetM, ...
    'scalpTri', scalpTri, ...
    'scalpVertices', vertices, ...
    'scalpOutwardNormals', normals);
end
