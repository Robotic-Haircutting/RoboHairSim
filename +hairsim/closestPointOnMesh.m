function [closestPointWorldM, closestFaceIndex, closestDistanceM] = ...
        closestPointOnMesh(pointWorldM, meshWorld)
%CLOSESTPOINTONMESH Nearest point on a triangulated mesh surface to a query point.
%   Shared by hairsim.control.applyMeshClipperSafetyGovernor (real-time
%   teleop safety clamp) and hairsim.evaluateClipperSphereMeshClearance
%   (clearance metrics), which previously each carried their own
%   byte-for-byte identical copy of this function.

faces = meshWorld.faces;
vertices = meshWorld.verticesWorld;
closestPointWorldM = [nan, nan, nan];
closestFaceIndex = 1;
closestDistanceSquaredM = inf;

for faceIndex = 1:size(faces, 1)
    tri = vertices(faces(faceIndex, :), :);
    candidatePoint = closestPointOnTriangle(pointWorldM, tri(1, :), tri(2, :), tri(3, :));
    candidateDistanceSquaredM = sum((pointWorldM - candidatePoint) .^ 2);
    if candidateDistanceSquaredM < closestDistanceSquaredM
        closestDistanceSquaredM = candidateDistanceSquaredM;
        closestPointWorldM = candidatePoint;
        closestFaceIndex = faceIndex;
    end
end

closestDistanceM = sqrt(closestDistanceSquaredM);
end

function closestPoint = closestPointOnTriangle(point, a, b, c)
ab = b - a;
ac = c - a;
ap = point - a;
d1 = dot(ab, ap);
d2 = dot(ac, ap);
if d1 <= 0 && d2 <= 0
    closestPoint = a;
    return;
end

bp = point - b;
d3 = dot(ab, bp);
d4 = dot(ac, bp);
if d3 >= 0 && d4 <= d3
    closestPoint = b;
    return;
end

vc = d1 * d4 - d3 * d2;
if vc <= 0 && d1 >= 0 && d3 <= 0
    v = d1 / (d1 - d3);
    closestPoint = a + v * ab;
    return;
end

cp = point - c;
d5 = dot(ab, cp);
d6 = dot(ac, cp);
if d6 >= 0 && d5 <= d6
    closestPoint = c;
    return;
end

vb = d5 * d2 - d1 * d6;
if vb <= 0 && d2 >= 0 && d6 <= 0
    w = d2 / (d2 - d6);
    closestPoint = a + w * ac;
    return;
end

va = d3 * d6 - d5 * d4;
if va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0
    w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    closestPoint = b + w * (c - b);
    return;
end

denom = 1 / (va + vb + vc);
v = vb * denom;
w = vc * denom;
closestPoint = a + ab * v + ac * w;
end
