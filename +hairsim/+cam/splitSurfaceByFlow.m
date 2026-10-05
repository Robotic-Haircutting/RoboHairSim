function [parts, info] = splitSurfaceByFlow(surface, options)
%SPLITSURFACEBYFLOW Split a machining surface whose hair flow diverges
%   (e.g. the crown: hair in front of it lies forward, behind it backward)
%   into two parts with coherent flow, so each can get its own
%   against-the-flow raster (hairsim.cam.generateRasterPasses uses ONE
%   flow direction per surface).
%
%   Coherence = length of the mean unit flow vector (1 = all roots flow the
%   same way, 0 = cancelling). Below options.MinResultant the roots are
%   split by the sign of their flow along the principal flow axis (the
%   axis of the flow vectors' second moment, which sees +d and -d alike).
%   Otherwise parts = {surface} unchanged.
%
%   parts: cell of surfaces (same fields; root arrays restricted, centre,
%   meanFlow and transverse recomputed; faces and headQuery shared).
%   info: resultant, split (logical), partSizes, axis.

arguments
    surface (1, 1) struct
    options.MinResultant (1, 1) double {mustBeInRange(options.MinResultant, 0, 1)} = 0.5
end

F = surface.flowDirs;
resultant = norm(mean(F, 1));
info = struct('resultant', resultant, 'split', false, 'partSizes', size(F, 1), 'axis', []);
parts = {surface};
if resultant >= options.MinResultant || size(F, 1) < 2
    return;
end

[~, ~, V] = svd(F, 'econ');
axis = V(:, 1)';
side = F * axis' >= 0;
if all(side) || ~any(side)
    return;
end
parts = {subSurface(surface, side), subSurface(surface, ~side)};
info.split = true;
info.partSizes = [nnz(side), nnz(~side)];
info.axis = axis;
end

function s = subSurface(surface, mask)
s = surface;
s.strandIdx = surface.strandIdx(mask);
s.needsCut = surface.needsCut(mask);
s.rootsWorldM = surface.rootsWorldM(mask, :);
s.rootNormals = surface.rootNormals(mask, :);
s.flowDirs = surface.flowDirs(mask, :);
s.centreWorldM = mean(s.rootsWorldM, 1);
meanFlow = mean(s.flowDirs, 1);
meanFlow = meanFlow - dot(meanFlow, s.meanNormal) * s.meanNormal;
s.meanFlow = meanFlow / norm(meanFlow);
s.transverse = cross(s.meanNormal, s.meanFlow);
end
