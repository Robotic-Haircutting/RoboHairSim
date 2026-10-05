function cutModel = createCombLiftCutModel(tool, options)
%CREATECOMBLIFTCUTMODEL Cut model for hairsim.hair.cutting.applyCombLiftCutToGuideHair.
%   The guard comb is logic, not geometry: when the clipper passes over a
%   strand's root close enough to the scalp (hairsim.eval.sweepCombLiftCoverage,
%   the same predicate the planner uses), the comb stands the strand up and
%   the blade cuts it at its target length -- haircutState.lengthGoal's
%   targetActiveNodeCounts (uniform or per region,
%   hairsim.hair.setRegionTargetLengthGoal). Strands already at or below
%   target are never touched.
%
%   tool: hairsim.tool.createClipperTool. The model remembers the previous
%   CUT tick's T_world_ee (lastToolTransform) so consecutive ticks are swept,
%   not sampled; hairsim.hair.updateHairLayerForClipper keeps it current.
%
%   FlowDirs: (numStrands,3) hair-flow direction per strand (as
%   hairsim.cam.buildMachiningSurface computes it). Required when
%   tool.maxFeedAngleRad is finite (feed-direction check); otherwise unused.

arguments
    tool (1, 1) struct
    options.FlowDirs (:, 3) double = zeros(0, 3)
end

cutModel = struct( ...
    'type', "combLift", ...
    'tool', tool, ...
    'lastToolTransform', [], ...
    'flowDirs', options.FlowDirs);
end
