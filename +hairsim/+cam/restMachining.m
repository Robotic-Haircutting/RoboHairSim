function [segments, covered, history] = restMachining(surface, segments, covered, tool, options)
%RESTMACHINING Rest machining + corner cleanup (sync point 1).
%   Repeats: cluster the roots that still need cutting and are not covered
%   (3-D single linkage, ClusterLinkM), and for the largest open cluster
%   generate short candidate passes through it -- feed against the local
%   hair flow and across it, each in the placements of
%   hairsim.cam.edgeVariants (centred, offset, rolled = "use the edge of
%   the clipper") -- and keep the candidate covering the most open roots.
%   A cluster no candidate can reach is given up and reported; the loop
%   ends when nothing is open or MaxIterations is hit.
%
%   covered: (M,1) logical over surface roots, updated. segments gain the
%   kept candidates (kind "rest" for centred, "edge" otherwise).
%   history: one row per iteration (clusterSize, gain, variant, feed).

arguments
    surface (1, 1) struct
    segments (1, :) struct
    covered (:, 1) logical
    tool (1, 1) struct
    options.ClusterLinkM (1, 1) double {mustBePositive} = 0.015
    options.MaxIterations (1, 1) double {mustBePositive, mustBeInteger} = 300
    options.EndMarginM (1, 1) double {mustBeNonnegative} = 0.008
    options.SampleStepM (1, 1) double {mustBePositive} = 0.0025
    options.OnProgress = []
end

variants = hairsim.cam.edgeVariants(tool);
givenUp = false(size(covered));
history = struct('clusterSize', {}, 'gain', {}, 'variant', {}, 'feed', {});
for iter = 1:options.MaxIterations
    open = find(surface.needsCut & ~covered & ~givenUp);
    if isempty(open)
        break;
    end
    clusters = hairsim.analysis.clusterResidualPoints3D(surface.rootsWorldM(open, :), options.ClusterLinkM);
    [~, order] = sort(cellfun(@numel, clusters), 'descend');
    members = open(clusters{order(1)});
    pts = surface.rootsWorldM(members, :);
    [~, ci] = min(vecnorm(pts - mean(pts, 1), 2, 2));
    centreRoot = members(ci);
    n = surface.rootNormals(centreRoot, :);
    flow = surface.flowDirs(centreRoot, :);
    feeds = [-flow; cross(n, flow)];

    best = struct('gain', 0, 'segments', [], 'variant', "", 'feed', 0);
    for f = 1:size(feeds, 1)
        feet = lineOnScalp(surface, surface.rootsWorldM(centreRoot, :), n, feeds(f, :), pts, ...
            options.EndMarginM, options.SampleStepM);
        if size(feet, 1) < 2
            continue;
        end
        for v = variants
            kind = "edge";
            if v.name == "centre"
                kind = "rest";
            end
            cand = hairsim.cam.buildSegments({feet}, surface, tool, kind, ...
                'LateralOffsetM', v.lateralOffsetM, 'RollRad', v.rollRad);
            if isempty(cand)
                continue;
            end
            hit = coverageOf(cand, surface.rootsWorldM(open, :), surface.flowDirs(open, :), tool);
            gain = nnz(hit);
            if gain > best.gain
                best = struct('gain', gain, 'segments', cand, 'variant', v.name, 'feed', f);
            end
            if nnz(hit(ismember(open, members))) == numel(members)
                break;                      % whole cluster covered: simplest placement wins
            end
        end
        if best.gain >= numel(members)
            break;
        end
    end
    history(end + 1) = struct('clusterSize', numel(members), 'gain', best.gain, ...
        'variant', best.variant, 'feed', best.feed); %#ok<AGROW>
    if best.gain == 0
        givenUp(members) = true;
        continue;
    end
    segments = [segments, best.segments]; %#ok<AGROW>
    covered(open(coverageOf(best.segments, surface.rootsWorldM(open, :), surface.flowDirs(open, :), tool))) = true;
    if ~isempty(options.OnProgress)
        options.OnProgress(sprintf("rest %s: iter %d, open %d, +%d (%s)", surface.region, iter, ...
            numel(open), best.gain, best.variant));
    end
end
end

function hit = coverageOf(segs, roots, flows, tool)
hit = false(size(roots, 1), 1);
for s = 1:numel(segs)
    hit = hit | hairsim.eval.sweepCombLiftCoverage(roots, segs(s).T_cut, tool, 'FlowDirs', flows);
end
end

function feet = lineOnScalp(surface, centre, n, feed, clusterPts, margin, step)
% straight line through the cluster in the tangent plane, projected onto the mesh
d = feed - dot(feed, n) * n;
d = d / norm(d);
s = (clusterPts - centre) * d';
a = min(s) - margin;
b = max(s) + margin;
q = (a:step:b)';
if numel(q) < 2
    q = [a; b];
end
line = centre + q * d;
[~, feet] = hairsim.tool.signedDistanceToHead(line, surface.headQuery, 0.05);
feet = feet(all(isfinite(feet), 2), :);
end
