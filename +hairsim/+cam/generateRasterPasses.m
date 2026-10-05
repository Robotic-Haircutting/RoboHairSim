function [passes, info] = generateRasterPasses(surface, headMeshWorld, tool, options)
%GENERATERASTERPASSES C2 of the camClipper plan: the main toolpath of one
%   region as parallel passes (CAM iso-planar raster), generated from
%   scratch -- no frozen lanes.
%
%   Cutting planes contain the region's mean hair flow and mean normal
%   (plane normal = surface.transverse), so every pass runs along the hair
%   flow. Each plane is intersected with the region's head-mesh faces
%   (hairsim.analysis.sliceMeshAtPlane), clipped to the machining boundary
%   (points within ClipM of a region root) and resampled every SampleStepM.
%   Feed direction = against the local hair flow.
%
%   Step-over: the blade covers 2*bladeHalfWidth = 26 mm across; the target
%   surface step-over is 26*(1 - OverlapFraction). Where the scalp turns
%   away from the plane stack (side of the head) the same plane offset
%   spreads further apart ON the surface, so the next offset is shrunk by
%   the local tilt factor sqrt(1 - (transverse . n)^2) (10th percentile of
%   the roots near the current plane) -- an iso-scallop-style adaptive
%   step-over. Anything still missed is left to rest machining.
%
%   passes: cell array of (K,3) foot-point polylines on the scalp, each in
%   feed order. info: planeOffsetsM, stepOverM (per plane).

arguments
    surface (1, 1) struct
    headMeshWorld (1, 1) struct
    tool (1, 1) struct
    options.OverlapFraction (1, 1) double {mustBeInRange(options.OverlapFraction, 0, 0.9)} = 0.3
    options.ClipM (1, 1) double {mustBePositive} = 0.012
    options.SampleStepM (1, 1) double {mustBePositive} = 0.0025
    options.MinPassLengthM (1, 1) double {mustBePositive} = 0.008
end

targetStep = 2 * tool.bladeHalfWidthM * (1 - options.OverlapFraction);
roots = surface.rootsWorldM;
u = (roots - surface.centreWorldM) * surface.transverse';
tiltFactor = sqrt(max(0, 1 - (surface.rootNormals * surface.transverse') .^ 2));
vertices = double(headMeshWorld.vertices);
faces = double(headMeshWorld.faces(surface.faces, :));

passes = {};
planeOffsets = [];
stepOvers = [];
u0 = min(u) + 0.5 * targetStep;
while u0 <= max(u) + 0.5 * targetStep
    planePoint = surface.centreWorldM + u0 * surface.transverse;
    polylines = hairsim.analysis.sliceMeshAtPlane(vertices, faces, planePoint, surface.transverse);
    for p = 1:numel(polylines)
        for run = clipToRegion(polylines{p}, roots, options.ClipM, options.MinPassLengthM)
            feet = resample(run{1}, options.SampleStepM);
            if size(feet, 1) < 2
                continue;
            end
            passes{end + 1} = orientAgainstFlow(feet, surface); %#ok<AGROW>
        end
    end
    near = abs(u - u0) <= targetStep;
    factor = 1;
    if any(near)
        factor = max(0.4, lowQuantile(tiltFactor(near), 0.1));
    end
    step = targetStep * factor;
    planeOffsets(end + 1) = u0; %#ok<AGROW>
    stepOvers(end + 1) = step; %#ok<AGROW>
    u0 = u0 + step;
end
info = struct('planeOffsetsM', planeOffsets, 'stepOverM', stepOvers, 'targetStepM', targetStep);
end

function runs = clipToRegion(polyline, roots, clipM, minLength)
keep = false(size(polyline, 1), 1);
for k = 1:size(polyline, 1)
    keep(k) = min(vecnorm(roots - polyline(k, :), 2, 2)) <= clipM;
end
runs = {};
edges = diff([false; keep; false]);
starts = find(edges == 1);
stops = find(edges == -1) - 1;
for r = 1:numel(starts)
    seg = polyline(starts(r):stops(r), :);
    if size(seg, 1) >= 2 && sum(vecnorm(diff(seg), 2, 2)) >= minLength
        runs{end + 1} = seg; %#ok<AGROW>
    end
end
end

function out = resample(polyline, step)
s = [0; cumsum(vecnorm(diff(polyline), 2, 2))];
[s, unique_] = unique(s);
polyline = polyline(unique_, :);
if numel(s) < 2
    out = polyline;
    return;
end
q = (0:step:s(end))';
if s(end) - q(end) > 0.25 * step
    q(end + 1) = s(end);
end
out = interp1(s, polyline, q);
end

function feet = orientAgainstFlow(feet, surface)
near = false(size(surface.rootsWorldM, 1), 1);
for k = 1:size(feet, 1)
    near = near | vecnorm(surface.rootsWorldM - feet(k, :), 2, 2) <= 0.015;
end
flow = surface.meanFlow;
if any(near)
    flow = mean(surface.flowDirs(near, :), 1);
end
if dot(feet(end, :) - feet(1, :), flow) > 0
    feet = flipud(feet);
end
end

function value = lowQuantile(x, p)
x = sort(x(:));
value = x(max(1, ceil(p * numel(x))));
end
