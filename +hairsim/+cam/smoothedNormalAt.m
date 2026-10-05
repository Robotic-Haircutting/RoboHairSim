function normals = smoothedNormalAt(pointsWorldM, headQuery, radiusM)
%SMOOTHEDNORMALAT Area-weighted mean of the head-mesh face normals within
%   radiusM of each point (outward, unit). The scanned mesh is noisy at
%   the triangle scale; a blade-scale radius (default use: 6 mm) gives the
%   surface normal the clipper actually rests on -- the CAM convention of
%   a smoothed drive-surface normal.

arguments
    pointsWorldM (:, 3) double
    headQuery (1, 1) struct
    radiusM (1, 1) double {mustBePositive} = 0.006
end

numPoints = size(pointsWorldM, 1);
normals = nan(numPoints, 3);
for k = 1:numPoints
    d = vecnorm(headQuery.centroids - pointsWorldM(k, :), 2, 2);
    near = d <= radiusM;
    if ~any(near)
        [~, near] = min(d);
    end
    n = sum(headQuery.normals(near, :) .* headQuery.areas(near), 1);
    normals(k, :) = n / norm(n);
end
end
