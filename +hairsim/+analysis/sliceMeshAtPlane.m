function polylines = sliceMeshAtPlane(vertices, faces, planePointWorldM, planeNormalWorld)
%SLICEMESHATPLANE Intersect a triangle mesh with an arbitrary plane
%   (point + normal) and stitch the resulting per-triangle segments into
%   ordered 3-D polylines along the true mesh surface.
%
%   Generalizes hairsim.analysis.sliceMeshAtHeight (a fixed z=const plane,
%   i.e. planeNormalWorld=[0,0,1]) to an arbitrary plane, needed for
%   regions whose hair-flow direction is not close to vertical --
%   scalp_top_left/top_right have X-dominated flow
%   (~[+-0.70,0.15,-0.18]), so a z-height reference slice for seed
%   placement would not run roughly perpendicular to their own flow the
%   way it does for scalp_back/scalp_left/scalp_right (all
%   Z-dominated, ~[*, *, -0.96 to -0.18]). sliceMeshAtHeight is
%   deliberately left untouched (a separate, independently-validated
%   function) rather than refactored into a thin wrapper over this one --
%   this file is a straight duplicate of its stitching logic, generalizing
%   only the per-triangle intersection test from a z-coordinate comparison
%   to a signed-distance-to-plane comparison.
%
%   vertices/faces: same convention as sliceMeshAtHeight (already
%   region-restricted; no notion of region here).
%   Returns polylines: cell array of (Mi,3) point sequences, longest
%   first -- same convention as sliceMeshAtHeight, same caveat that this
%   is a cross-section arc, not a surface geodesic.

planeNormalWorld = planeNormalWorld(:)' / norm(planeNormalWorld);
signedDist = @(p) (p - planePointWorldM(:)') * planeNormalWorld';

edgeOrder = [1, 2; 2, 3; 3, 1];
segments = zeros(0, 6);
tol = 1e-9;

for faceIdx = 1:size(faces, 1)
    v = vertices(faces(faceIdx, :), :);
    d = signedDist(v);
    pts = zeros(0, 3);
    for edgeIdx = 1:3
        ia = edgeOrder(edgeIdx, 1);
        ib = edgeOrder(edgeIdx, 2);
        da = d(ia);
        db = d(ib);
        if abs(da) <= tol
            pts = [pts; v(ia, :)]; %#ok<AGROW>
        elseif da * db < 0
            t = da / (da - db);
            pts = [pts; v(ia, :) + t * (v(ib, :) - v(ia, :))]; %#ok<AGROW>
        end
    end
    if isempty(pts)
        continue;
    end
    pts = uniquetol(pts, 1e-8, 'ByRows', true, 'DataScale', 1);
    if size(pts, 1) == 2
        segments(end + 1, :) = [pts(1, :), pts(2, :)]; %#ok<AGROW>
    end
end

polylines = stitchSegmentsToPolylines(segments);
end

function polylines = stitchSegmentsToPolylines(segments)
if isempty(segments)
    polylines = {};
    return;
end

endpointsA = segments(:, 1:3);
endpointsB = segments(:, 4:6);
allPoints = [endpointsA; endpointsB];
[uniquePoints, ~, nodeIds] = uniquetol(allPoints, 1e-8, 'ByRows', true, 'DataScale', 1);
numSegments = size(segments, 1);
nodeA = nodeIds(1:numSegments);
nodeB = nodeIds(numSegments + 1:end);

numNodes = size(uniquePoints, 1);
adjacency = cell(numNodes, 1);
for segIdx = 1:numSegments
    adjacency{nodeA(segIdx)} = [adjacency{nodeA(segIdx)}, nodeB(segIdx)];
    adjacency{nodeB(segIdx)} = [adjacency{nodeB(segIdx)}, nodeA(segIdx)];
end

usedSegment = false(numSegments, 1);
segmentLookup = containers.Map('KeyType', 'char', 'ValueType', 'double');
for segIdx = 1:numSegments
    key1 = sprintf('%d_%d', nodeA(segIdx), nodeB(segIdx));
    key2 = sprintf('%d_%d', nodeB(segIdx), nodeA(segIdx));
    segmentLookup(key1) = segIdx;
    segmentLookup(key2) = segIdx;
end

polylines = {};
degree = cellfun(@numel, adjacency);
startNodes = [find(degree == 1); find(degree ~= 1 & degree > 0)];

for startIdx = 1:numel(startNodes)
    startNode = startNodes(startIdx);
    neighbors = adjacency{startNode};
    for neighborIdx = 1:numel(neighbors)
        firstNext = neighbors(neighborIdx);
        key = sprintf('%d_%d', startNode, firstNext);
        if ~isKey(segmentLookup, key) || usedSegment(segmentLookup(key))
            continue;
        end
        [pathNodeIds, consumedSegments] = walkPolyline( ...
            startNode, firstNext, adjacency, segmentLookup, usedSegment);
        usedSegment(consumedSegments) = true;
        polylines{end + 1} = uniquePoints(pathNodeIds, :); %#ok<AGROW>
    end
end

lengths = cellfun(@(p) size(p, 1), polylines);
[~, order] = sort(lengths, 'descend');
polylines = polylines(order);
end

function [pathNodeIds, consumedSegments] = walkPolyline( ...
        startNode, firstNext, adjacency, segmentLookup, usedSegment)
pathNodeIds = startNode;
consumedSegments = zeros(0, 1);
currentNode = startNode;
nextNode = firstNext;
while true
    key = sprintf('%d_%d', currentNode, nextNode);
    if ~isKey(segmentLookup, key)
        break;
    end
    segIdx = segmentLookup(key);
    if usedSegment(segIdx) || any(consumedSegments == segIdx)
        break;
    end
    consumedSegments(end + 1) = segIdx; %#ok<AGROW>
    pathNodeIds(end + 1) = nextNode; %#ok<AGROW>
    if nextNode == startNode
        break;
    end
    neighbors = adjacency{nextNode};
    candidateNext = neighbors(neighbors ~= currentNode);
    unusedCandidate = [];
    for candIdx = 1:numel(candidateNext)
        candKey = sprintf('%d_%d', nextNode, candidateNext(candIdx));
        candSeg = segmentLookup(candKey);
        if ~usedSegment(candSeg) && ~any(consumedSegments == candSeg)
            unusedCandidate = candidateNext(candIdx);
            break;
        end
    end
    if isempty(unusedCandidate)
        break;
    end
    currentNode = nextNode;
    nextNode = unusedCandidate;
end
end
