function displayMesh = createClipperDisplayMesh(stlPath, toolMount, options)
%CREATECLIPPERDISPLAYMESH Display-only clipper head mesh in the EE frame.
%   Crops the blade-head wedge (the "triangular cone" at the blade end)
%   out of a full hair-clipper STL, decimates it, and places it in the
%   end-effector frame.
%
%   DISPLAY ONLY: cutting, clearance and collision use the parametric
%   clipper model (toolMount.tool, hairsim.tool.createClipperTool); nothing
%   here feeds planning, IK, cutting or collision.
%
%   Frame convention:
%   EE x = travel direction, EE z = into the scalp (-n), EE y = transverse.
%   The STL is expected with its long axis along +z (blade at max z),
%   blade width along x and wedge thickness along y; it is mapped
%       STL x -> EE y (blade edge perpendicular to travel),
%       STL y -> EE -x, STL z -> EE z (blade toward the scalp).
%   Scale: the blade-teeth width is set to the clipper model's blade width
%   (2 * tool.bladeHalfWidthM), and the teeth top-centre is placed at the
%   model's TCP (tool.tcpOffsetM along EE z), so the drawn blade is the
%   blade the comb-lift cut model uses. The gap between
%   the EE flange and the wedge base is filled by a plain cylindrical
%   adapter (adapterFaces/adapterVerticesEe).
%
%   displayMesh fields: faces, verticesEe (wedge), adapterFaces,
%   adapterVerticesEe, bladeFaces, bladeVerticesEe (thin quad on the
%   teeth plane = the cutting face, drawn colour-coded by cutFlag),
%   scale, bladeWidthM, stlPath.
%
%   Results are memoised per (stlPath, options) for the MATLAB session.

arguments
    stlPath (1, 1) string
    toolMount (1, 1) struct
    options.CropZFraction (1, 1) double = 0.8275    % keep z >= 0.96 of a 1.16-long model
    options.TeethSlabFraction (1, 1) double = 0.0175 % top slab used to measure the teeth
    options.TargetFaceCount (1, 1) double {mustBePositive} = 4000
    options.FlipTeeth (1, 1) logical = false        % rotate 180 deg about EE z
    options.AdapterRadiusM (1, 1) double {mustBePositive} = 0.008
end

persistent cache
if isempty(cache)
    cache = containers.Map('KeyType', 'char', 'ValueType', 'any');
end
fileInfo = dir(stlPath);
if isempty(fileInfo)
    error("hairsim:ClipperDisplayStlMissing", "Clipper display STL not found: %s", stlPath);
end
key = sprintf('%s|%s|%g|%g|%g|%d|%g|%g|%g', stlPath, fileInfo.date, options.CropZFraction, ...
    options.TeethSlabFraction, options.TargetFaceCount, options.FlipTeeth, options.AdapterRadiusM, ...
    toolMount.tool.bladeHalfWidthM, toolMount.tool.tcpOffsetM);
if isKey(cache, key)
    displayMesh = cache(key);
    return;
end

tri = stlread(stlPath);
points = tri.Points;
faces = tri.ConnectivityList;
zMin = min(points(:, 3));
zMax = max(points(:, 3));
zLength = zMax - zMin;

% ---- crop the head wedge (every vertex of a kept face above the cut)
zCut = zMin + options.CropZFraction * zLength;
pointZ = points(:, 3);
keepFace = all(pointZ(faces) >= zCut, 2);
faces = faces(keepFace, :);
[usedIdx, ~, newIdx] = unique(faces(:));
points = points(usedIdx, :);
faces = reshape(newIdx, [], 3);

% ---- measure the teeth (top slab) before decimation
inSlab = points(:, 3) >= zMax - options.TeethSlabFraction * zLength;
teethMin = min(points(inSlab, 1:2), [], 1);
teethMax = max(points(inSlab, 1:2), [], 1);
teethWidthStl = teethMax(1) - teethMin(1);
teethDepthStl = teethMax(2) - teethMin(2);
anchorStl = [0.5 * (teethMin + teethMax), zMax];

% ---- decimate (100k faces is far too heavy to redraw every tick)
reduced = reducepatch(struct('faces', faces, 'vertices', points), ...
    min(1, options.TargetFaceCount / size(faces, 1)));
faces = reduced.faces;
points = reduced.vertices;

% ---- STL frame -> EE frame
bladeWidthM = 2 * toolMount.tool.bladeHalfWidthM;
scale = bladeWidthM / teethWidthStl;
R_ee_stl = [0, -1, 0; 1, 0, 0; 0, 0, 1];
if options.FlipTeeth
    R_ee_stl = diag([-1, -1, 1]) * R_ee_stl;
end
tipZEeM = toolMount.tool.tcpOffsetM;
toEe = @(p) (scale * (p - anchorStl)) * R_ee_stl' + [0, 0, tipZEeM];
verticesEe = toEe(points);

% ---- cutting face: thin quad on the teeth plane
halfW = 0.5 * teethWidthStl;
halfD = 0.5 * teethDepthStl;
bladeCornersStl = anchorStl + [-halfW, -halfD, 0; halfW, -halfD, 0; halfW, halfD, 0; -halfW, halfD, 0];
bladeVerticesEe = toEe(bladeCornersStl) + [0, 0, 1e-4];

% ---- adapter cylinder from the flange (EE z = 0) to the wedge base
wedgeBaseZ = min(verticesEe(:, 3));
[cx, cy, cz] = cylinder(options.AdapterRadiusM, 16);
cz = cz * (wedgeBaseZ + 0.001);
adapter = surf2patch(cx, cy, cz, 'triangles');

displayMesh = struct( ...
    'faces', faces, 'verticesEe', verticesEe, ...
    'adapterFaces', adapter.faces, 'adapterVerticesEe', adapter.vertices, ...
    'bladeFaces', [1, 2, 3, 4], 'bladeVerticesEe', bladeVerticesEe, ...
    'scale', scale, 'bladeWidthM', bladeWidthM, 'stlPath', stlPath);
cache(key) = displayMesh;
end
