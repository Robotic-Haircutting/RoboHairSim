function [activeNodeCounts, cutInfo] = applyCombLiftCutToGuideHair( ...
        guideX, guideY, guideZ, activeNodeCounts, targetActiveNodeCounts, T_world_tcp, tool, options)
%APPLYCOMBLIFTCUTTOGUIDEHAIR Comb-lift clipper cut (see createCombLiftCutModel).
%   T_world_tcp: (4,4,N) TCP poses swept this tick (the previous CUT tick's
%   pose followed by the current one, or just the current one). Each strand
%   whose root (node 1) the pass covers and which is still longer than its
%   target is cut to exactly targetActiveNodeCounts.

arguments
    guideX (:, :) double
    guideY (:, :) double
    guideZ (:, :) double
    activeNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative}
    targetActiveNodeCounts (1, :) double {mustBeInteger, mustBeNonnegative}
    T_world_tcp (4, 4, :) double
    tool (1, 1) struct
    options.FlowDirs (:, 3) double = zeros(0, 3)   % per strand; see sweepCombLiftCoverage
end

hairsim.hair.validateGuideMatrices(guideX, guideY, guideZ, activeNodeCounts);
needsCut = find(activeNodeCounts > targetActiveNodeCounts);
strandIndices = zeros(1, 0);
if ~isempty(needsCut)
    roots = [guideX(1, needsCut)', guideY(1, needsCut)', guideZ(1, needsCut)'];
    flows = zeros(0, 3);
    if ~isempty(options.FlowDirs)
        flows = options.FlowDirs(needsCut, :);
    end
    covered = hairsim.eval.sweepCombLiftCoverage(roots, T_world_tcp, tool, 'FlowDirs', flows);
    strandIndices = needsCut(covered);
end
nodeIndices = targetActiveNodeCounts(strandIndices) + 1;
activeNodeCounts(strandIndices) = targetActiveNodeCounts(strandIndices);

cutInfo = struct();
cutInfo.cutCount = numel(strandIndices);
cutInfo.strandIndices = strandIndices;
cutInfo.nodeIndices = nodeIndices;
end
