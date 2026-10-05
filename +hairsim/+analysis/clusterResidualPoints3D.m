function clusters = clusterResidualPoints3D(pointsWorldM, linkM)
%CLUSTERRESIDUALPOINTS3D Single-linkage 3D clustering: connect two points
%   whenever they are within linkM of each other; return the connected
%   components as index groups into pointsWorldM. Used by
%   hairsim.analysis.refineHairCoverageStage2 to decide how many local
%   streamlines to seed for a set of residual (still-uncovered) hairs --
%   one streamline per contiguous 3D gap cluster, not one per hair.

arguments
    pointsWorldM (:, 3) double
    linkM (1, 1) double {mustBePositive}
end

n = size(pointsWorldM, 1);
adjacency = false(n);
for i = 1:n
    for j = (i + 1):n
        if norm(pointsWorldM(i, :) - pointsWorldM(j, :)) <= linkM
            adjacency(i, j) = true;
            adjacency(j, i) = true;
        end
    end
end

visited = false(n, 1);
clusters = {};
for i = 1:n
    if visited(i)
        continue;
    end
    stack = i;
    component = [];
    while ~isempty(stack)
        current = stack(end);
        stack(end) = [];
        if visited(current)
            continue;
        end
        visited(current) = true;
        component(end + 1) = current; %#ok<AGROW>
        neighbors = find(adjacency(current, :));
        stack = [stack, neighbors(~visited(neighbors))]; %#ok<AGROW>
    end
    clusters{end + 1} = component; %#ok<AGROW>
end
end
