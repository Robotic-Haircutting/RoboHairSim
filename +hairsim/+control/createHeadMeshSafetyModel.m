function [meshModel, cacheCreated] = createHeadMeshSafetyModel(headMeshLocal, cachePath)
%CREATEHEADMESHSAFETYMODEL Load or generate cached STL surface safety data.

if nargin < 2
    cachePath = "";
end
cachePath = char(cachePath);
if ~isempty(cachePath) && isfile(cachePath)
    meshModel = readMeshSafetyCache(cachePath);
    cacheCreated = false;
    return;
end

verticesLocal = headMeshLocal.vertices - headMeshLocal.center;
faces = double(headMeshLocal.faces);
[faceNormalsLocal, faceCentersLocal] = createFaceGeometry(verticesLocal, faces);

meshModel = struct();
meshModel.verticesLocal = verticesLocal;
meshModel.faces = faces;
meshModel.faceNormalsLocal = faceNormalsLocal;
meshModel.faceCentersLocal = faceCentersLocal;
meshModel.coordinateFrame = "centered_head_local";
meshModel.units = "meters";
meshModel.sourcePath = char(headMeshLocal.sourcePath);

if ~isempty(cachePath)
    writeMeshSafetyCache(cachePath, meshModel);
end
cacheCreated = ~isempty(cachePath);
end

function [faceNormals, faceCenters] = createFaceGeometry(vertices, faces)
v1 = vertices(faces(:, 1), :);
v2 = vertices(faces(:, 2), :);
v3 = vertices(faces(:, 3), :);
faceNormals = cross(v2 - v1, v3 - v1, 2);
normalLengths = vecnorm(faceNormals, 2, 2);
validNormals = normalLengths > eps;
faceNormals(validNormals, :) = faceNormals(validNormals, :) ./ normalLengths(validNormals);
faceNormals(~validNormals, :) = repmat([1, 0, 0], nnz(~validNormals), 1);
faceCenters = (v1 + v2 + v3) / 3;
end

function meshModel = readMeshSafetyCache(cachePath)
payload = jsondecode(fileread(cachePath));
meshModel = struct();
meshModel.verticesLocal = ensureThreeColumnArray(double(payload.verticesLocal));
meshModel.faces = ensureThreeColumnArray(double(payload.faces));
meshModel.faceNormalsLocal = ensureThreeColumnArray(double(payload.faceNormalsLocal));
meshModel.faceCentersLocal = ensureThreeColumnArray(double(payload.faceCentersLocal));
meshModel.coordinateFrame = string(payload.coordinateFrame);
meshModel.units = string(payload.units);
meshModel.sourcePath = string(payload.sourcePath);
end

function writeMeshSafetyCache(cachePath, meshModel)
cacheDir = fileparts(cachePath);
if ~isempty(cacheDir) && ~isfolder(cacheDir)
    mkdir(cacheDir);
end
payload = struct();
payload.sourcePath = char(meshModel.sourcePath);
payload.units = char(meshModel.units);
payload.coordinateFrame = char(meshModel.coordinateFrame);
payload.verticesLocal = meshModel.verticesLocal;
payload.faces = meshModel.faces;
payload.faceNormalsLocal = meshModel.faceNormalsLocal;
payload.faceCentersLocal = meshModel.faceCentersLocal;
payload.vertexCount = size(meshModel.verticesLocal, 1);
payload.faceCount = size(meshModel.faces, 1);

fid = fopen(cachePath, 'w');
if fid < 0
    error("hairsim:MeshSafetyCacheWriteFailed", ...
        "Cannot write head mesh safety cache: %s", cachePath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', jsonencode(payload));
end

function values = ensureThreeColumnArray(values)
if size(values, 2) ~= 3
    values = reshape(values, [], 3);
end
end
