function cutModel = createGuardBladeCutModel(headMeshWorld, toolMount, guardHeightM, options)
%CREATEGUARDBLADECUTMODEL Cut model for hairsim.hair.cutting.applyGuardBladeCutToGuideHair.
%   Flat blade matching the display-only clipper head
%   (hairsim.motion.createClipperDisplayMesh): blade width = cutting-sphere
%   diameter, blade plane at the sphere's far point (toolTipOffset +
%   toolRadiusM along EE z, ~2 mm from the scalp along a planned path).
%   guardHeightM = guard-comb height = target contour offset from the scalp.
%
%   Scalp height of a point = (point - nearest scalp vertex) . outward
%   vertex normal; normals are flipped to point away from the head
%   centroid (the head is star-shaped about it). Checked against the
%   planner's own sphere clearance on the rail bundle: median 1.99 mm vs
%   the planned 2 mm.

arguments
    headMeshWorld (1, 1) struct
    toolMount (1, 1) struct
    guardHeightM (1, 1) double {mustBeNonnegative}
    options.BladeHalfDepthM (1, 1) double {mustBePositive} = 0.005
    options.BladeHeightM (1, 1) double {mustBePositive} = 2 * toolMount.toolRadiusM
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
    'bladeHalfWidthM', toolMount.toolRadiusM, ...
    'bladeHalfDepthM', options.BladeHalfDepthM, ...
    'bladeHeightM', options.BladeHeightM, ...
    'bladeTipZM', toolMount.T_ee_tool_center(3, 4) + toolMount.toolRadiusM, ...
    'scalpTri', scalpTri, ...
    'scalpVertices', vertices, ...
    'scalpOutwardNormals', normals);
end
