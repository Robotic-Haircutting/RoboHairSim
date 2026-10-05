function [hullModel, cacheCreated] = createHeadConvexHullSafetyModel(headMeshLocal, cachePath)
%CREATEHEADCONVEXHULLSAFETYMODEL Load or generate a cached head hull model.
%   Simplified proxy used ONLY for hair-physics constraints (hair strands vs head).
%   It must not be used for robot-head or tool-head collision checking: robot-head
%   uses hairsim.motion.createHeadCollisionModel, tool-head uses the exact mesh.

cachePath = char(cachePath);
if isfile(cachePath)
    hullModel = readHullCache(cachePath);
    cacheCreated = false;
    return;
end

verticesLocal = headMeshLocal.vertices - headMeshLocal.center;
verticesLocal = unique(verticesLocal, 'rows');
hullFaces = convhulln(verticesLocal);
centroidLocal = mean(verticesLocal, 1);

normalsLocal = zeros(size(hullFaces, 1), 3);
offsetsLocalM = zeros(size(hullFaces, 1), 1);
isValidPlane = false(size(hullFaces, 1), 1);

for planeIndex = 1:size(hullFaces, 1)
    facetVertices = verticesLocal(hullFaces(planeIndex, :), :);
    edgeA = facetVertices(2, :) - facetVertices(1, :);
    edgeB = facetVertices(3, :) - facetVertices(1, :);
    normalLocal = cross(edgeA, edgeB);
    normalLength = norm(normalLocal);
    if normalLength <= eps
        continue;
    end
    normalLocal = normalLocal / normalLength;
    if dot(normalLocal, centroidLocal - facetVertices(1, :)) > 0
        normalLocal = -normalLocal;
    end
    normalsLocal(planeIndex, :) = normalLocal;
    offsetsLocalM(planeIndex) = -dot(normalLocal, facetVertices(1, :));
    isValidPlane(planeIndex) = true;
end

hullModel = struct();
hullModel.normalsLocal = normalsLocal(isValidPlane, :);
hullModel.offsetsLocalM = offsetsLocalM(isValidPlane);
hullModel.coordinateFrame = "centered_head_local";
hullModel.units = "meters";
hullModel.sourcePath = char(headMeshLocal.sourcePath);

writeHullCache(cachePath, hullModel);
cacheCreated = true;
end

function hullModel = readHullCache(cachePath)
payload = jsondecode(fileread(cachePath));
hullModel = struct();
hullModel.normalsLocal = double(payload.normalsLocal);
hullModel.offsetsLocalM = double(payload.offsetsLocalM(:));
if size(hullModel.normalsLocal, 2) ~= 3
    hullModel.normalsLocal = reshape(hullModel.normalsLocal, [], 3);
end
hullModel.coordinateFrame = string(payload.coordinateFrame);
hullModel.units = string(payload.units);
hullModel.sourcePath = string(payload.sourcePath);
end

function writeHullCache(cachePath, hullModel)
cacheDir = fileparts(cachePath);
if ~isempty(cacheDir) && ~isfolder(cacheDir)
    mkdir(cacheDir);
end
payload = struct();
payload.sourcePath = char(hullModel.sourcePath);
payload.units = char(hullModel.units);
payload.coordinateFrame = char(hullModel.coordinateFrame);
payload.normalsLocal = hullModel.normalsLocal;
payload.offsetsLocalM = hullModel.offsetsLocalM;
payload.planeCount = size(hullModel.normalsLocal, 1);

fid = fopen(cachePath, 'w');
if fid < 0
    error("hairsim:HullCacheWriteFailed", ...
        "Cannot write head convex hull cache: %s", cachePath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', jsonencode(payload));
end
