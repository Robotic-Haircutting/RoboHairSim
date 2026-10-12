function [inside, distanceM] = regionMembership(teleopState, points)
%REGIONMEMBERSHIP Which points lie in Teleoperation's selected region.
%   A point is inside when the hair root closest to it belongs to the
%   region AND is within teleopState.regionReachM of it (so the forehead,
%   ears and neck next to a region are not part of it). distanceM: distance
%   from each point to the closest root of the region. With no region
%   selected (regionName = "") every point is inside, distance 0.
%   The region overlay (hairsim.app.view.drawTeleopRegion) and the move
%   limit (hairsim.app.teleop.stepTeleop) both use this test, so what is
%   highlighted is exactly where the clipper may go.

arguments
    teleopState (1, 1) struct
    points (:, 3) double
end

numPoints = size(points, 1);
inside = true(numPoints, 1);
distanceM = zeros(numPoints, 1);
if strlength(teleopState.regionName) == 0
    return;
end
roots = teleopState.hairRoots;
isRegionRoot = teleopState.strandRegions(:) == teleopState.regionName;
rootSq = sum(roots .^ 2, 2)';
chunk = 2000;                  % rows of the point-to-root distance matrix at a time
for first = 1:chunk:numPoints
    rows = first:min(first + chunk - 1, numPoints);
    P = points(rows, :);
    D2 = max(0, sum(P .^ 2, 2) + rootSq - 2 * (P * roots'));
    [nearestSq, nearest] = min(D2, [], 2);
    inside(rows) = isRegionRoot(nearest) & sqrt(nearestSq) <= teleopState.regionReachM;
    distanceM(rows) = sqrt(min(D2(:, isRegionRoot), [], 2));
end
end
