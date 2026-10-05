function headMesh = loadHeadMeshMeters(stlPath)
%LOADHEADMESHMETERS Load a binary STL head mesh and scale vertices to meters.

triangles = readBinaryStl(stlPath);
vertices = reshape(permute(triangles, [2, 1, 3]), [], 3);
faces = reshape(1:size(vertices, 1), 3, []).';

bboxMinOriginal = min(vertices, [], 1);
bboxMaxOriginal = max(vertices, [], 1);
maxExtentOriginal = max(bboxMaxOriginal - bboxMinOriginal);
scaleToMeters = 1.0;
if maxExtentOriginal > 1.0
    scaleToMeters = 1e-3;
    vertices = vertices * scaleToMeters;
end

bboxMin = min(vertices, [], 1);
bboxMax = max(vertices, [], 1);

headMesh = struct();
headMesh.faces = faces;
headMesh.vertices = vertices;
headMesh.sourcePath = char(stlPath);
headMesh.scaleToMeters = scaleToMeters;
headMesh.bboxMin = bboxMin;
headMesh.bboxMax = bboxMax;
headMesh.bboxSize = bboxMax - bboxMin;
headMesh.center = (bboxMin + bboxMax) / 2;
end

function triangles = readBinaryStl(stlPath)
fid = fopen(stlPath, "rb");
if fid < 0
    error("hairsim:MissingHeadStl", "Cannot open STL file: %s", stlPath);
end
cleanup = onCleanup(@() fclose(fid));

fseek(fid, 80, "bof");
nTriangles = fread(fid, 1, "uint32=>double", 0, "ieee-le");
raw = fread(fid, [50, nTriangles], "uint8=>uint8");
if size(raw, 2) ~= nTriangles
    error("hairsim:InvalidHeadStl", "Incomplete binary STL file: %s", stlPath);
end

values = typecast(reshape(raw(13:48, :), [], 1), "single");
triangles = permute(reshape(values, 3, 3, []), [3, 2, 1]);
triangles = double(triangles);
end
