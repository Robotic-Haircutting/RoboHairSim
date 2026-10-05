function [distanceM, closestPointsM, faceIdx] = signedDistanceToHead(pointsWorldM, query, capM)
%SIGNEDDISTANCETOHEAD Exact signed distance from points to the head mesh.
%   query: hairsim.tool.createHeadDistanceQuery. Positive = outside.
%   capM (optional, default Inf): distances are only resolved up to capM --
%   a point with no face closer than capM returns +capM (outside, far
%   enough) and NaN closest point/face. Callers that only test clearances
%   of a few millimetres pass a small cap, which lets one centroid cull
%   reject almost every face.

arguments
    pointsWorldM (:, 3) double
    query (1, 1) struct
    capM (1, 1) double {mustBePositive} = Inf
end

numPoints = size(pointsWorldM, 1);
distanceM = repmat(capM, numPoints, 1);
closestPointsM = nan(numPoints, 3);
faceIdx = nan(numPoints, 1);
if numPoints == 0
    return;
end

% ---- one cull for the whole batch: faces near the points' bounding box
lo = min(pointsWorldM, [], 1);
hi = max(pointsWorldM, [], 1);
centre = 0.5 * (lo + hi);
halfDiag = 0.5 * norm(hi - lo);
reach = min(capM, 1);
near = vecnorm(query.centroids - centre, 2, 2) - query.radii <= halfDiag + reach;
if ~isfinite(capM)
    near = true(size(query.radii));
end
faces = find(near);
if isempty(faces)
    return;
end
A = query.A(faces, :);
B = query.B(faces, :);
C = query.C(faces, :);
N = query.normals(faces, :);
cen = query.centroids(faces, :);
rad = query.radii(faces);

chunk = max(1, floor(4e6 / numel(faces)));
for first = 1:chunk:numPoints
    rows = first:min(numPoints, first + chunk - 1);
    for r = rows
        p = pointsWorldM(r, :);
        dc = vecnorm(cen - p, 2, 2);
        upper = min(dc + rad);
        cand = find(dc - rad <= min(upper, capM));
        if isempty(cand)
            continue;
        end
        q = closestOnTriangles(p, A(cand, :), B(cand, :), C(cand, :));
        d = vecnorm(q - p, 2, 2);
        [dMin, k] = min(d);
        if dMin > capM
            continue;
        end
        sgn = sign(dot(p - q(k, :), N(cand(k), :)));
        if sgn == 0
            sgn = 1;
        end
        distanceM(r) = sgn * dMin;
        closestPointsM(r, :) = q(k, :);
        faceIdx(r) = faces(cand(k));
    end
end
end

function q = closestOnTriangles(p, a, b, c)
% Vectorised closest point on each triangle (Ericson, Real-Time Collision
% Detection, 5.1.5) for one query point p against many triangles.
ab = b - a;
ac = c - a;
ap = p - a;
d1 = sum(ab .* ap, 2);
d2 = sum(ac .* ap, 2);
bp = p - b;
d3 = sum(ab .* bp, 2);
d4 = sum(ac .* bp, 2);
cp = p - c;
d5 = sum(ab .* cp, 2);
d6 = sum(ac .* cp, 2);
va = d3 .* d6 - d5 .* d4;
vb = d5 .* d2 - d1 .* d6;
vc = d1 .* d4 - d3 .* d2;

denom = va + vb + vc;
denom(abs(denom) < eps) = eps;
v = vb ./ denom;
w = vc ./ denom;
q = a + ab .* v + ac .* w;                               % interior

m = vc <= 0 & d1 >= 0 & d3 <= 0;                         % edge AB
t = reshape(d1(m) ./ max(d1(m) - d3(m), eps), [], 1);
q(m, :) = a(m, :) + ab(m, :) .* t;
m = vb <= 0 & d2 >= 0 & d6 <= 0;                         % edge AC
t = reshape(d2(m) ./ max(d2(m) - d6(m), eps), [], 1);
q(m, :) = a(m, :) + ac(m, :) .* t;
m = va <= 0 & (d4 - d3) >= 0 & (d5 - d6) >= 0;           % edge BC
t = reshape((d4(m) - d3(m)) ./ max((d4(m) - d3(m)) + (d5(m) - d6(m)), eps), [], 1);
q(m, :) = b(m, :) + (c(m, :) - b(m, :)) .* t;
m = d1 <= 0 & d2 <= 0;                                   % vertex A
q(m, :) = a(m, :);
m = d3 >= 0 & d4 <= d3;                                  % vertex B
q(m, :) = b(m, :);
m = d6 >= 0 & d5 <= d6;                                  % vertex C
q(m, :) = c(m, :);
end
