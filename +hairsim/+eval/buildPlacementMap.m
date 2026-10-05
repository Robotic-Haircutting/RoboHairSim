function placement = buildPlacementMap(surface, tool, options)
%BUILDPLACEMENTMAP Robot-free "can the clipper be placed here" map of one
%   region (line B deliverable): for every root that needs cutting, try the
%   clipper placements of hairsim.cam.edgeVariants with the feed against
%   the hair flow and then across it; a root is PLACEABLE when some
%   placement clears the head (hairsim.tool.resolveContactStandoff) and
%   covers the root (hairsim.eval.sweepCombLiftCoverage). C_place, the share
%   of placeable roots, is the upper bound of the ideal coverage C_A.
%
%   placement fields (one row per region root; non-cut roots are NaN/""):
%   placeable (logical), how (variant name, or "" if none), minClearanceM
%   (best clearance seen when none fits), C_place (over roots needing cut).

arguments
    surface (1, 1) struct
    tool (1, 1) struct
    options.OnProgress = []
end

variants = hairsim.cam.edgeVariants(tool);
numRoots = size(surface.rootsWorldM, 1);
placeable = false(numRoots, 1);
how = strings(numRoots, 1);
bestClearance = nan(numRoots, 1);
todo = find(surface.needsCut)';
for count = 1:numel(todo)
    k = todo(count);
    [~, foot] = hairsim.tool.signedDistanceToHead(surface.rootsWorldM(k, :), surface.headQuery, 0.05);
    if any(isnan(foot))
        foot = surface.rootsWorldM(k, :);
    end
    n = surface.rootNormals(k, :);
    feeds = [-surface.flowDirs(k, :); cross(n, surface.flowDirs(k, :))];
    best = -inf;
    for f = 1:size(feeds, 1)
        for v = variants
            standoff = tool.nominalStandoffM + tool.bladeHalfWidthM * abs(sin(v.rollRad));
            Tn = hairsim.tool.buildTcpPose(foot, n, feeds(f, :), tool, 'StandoffM', standoff, ...
                'LateralOffsetM', v.lateralOffsetM, 'RollRad', v.rollRad);
            [T, ok, ~, clr] = hairsim.tool.resolveContactStandoff(Tn, n, tool, surface.headQuery);
            best = max(best, clr.minM);
            if ok && hairsim.eval.sweepCombLiftCoverage(surface.rootsWorldM(k, :), T, tool, ...
                    'FlowDirs', surface.flowDirs(k, :))
                placeable(k) = true;
                how(k) = v.name;
                break;
            end
        end
        if placeable(k)
            break;
        end
    end
    if ~placeable(k)
        bestClearance(k) = best;
    end
    if ~isempty(options.OnProgress) && mod(count, 50) == 0
        options.OnProgress(sprintf("placement %s: %d/%d", surface.region, count, numel(todo)));
    end
end
placement = struct('placeable', placeable, 'how', how, 'minClearanceM', bestClearance, ...
    'C_place', mean(placeable(surface.needsCut)));
end
