function regionPlan = camClipperPlanner(haircutState, regionName, targetLengthM, context, options)
%CAMCLIPPERPLANNER The camClipper region planner (registered as
%   "camClipper" by hairsim.planning.registerBuiltinPresets): clipper tool
%   model, CAM toolpaths generated from scratch (hairsim.cam), robot
%   post-processing with a per-segment rail-station search (hairsim.post),
%   and the ideal-vs-robot evaluation (hairsim.eval). Uses no frozen lanes.
%
%   Same signature as every registered planner. targetLengthM is this
%   region's guard-comb length. The returned regionPlan follows the shared
%   contract (one stroke per executable segment piece) and additionally
%   carries regionPlan.camClipper: toolpathPlan (the tool-position file),
%   comparison (C_place, C_A, C_exec, C_B, loss breakdown) and the per-piece
%   post-processing outcomes.
%
%   Rail: with a rail placement (context.railContextBuilder) stations are
%   searched per segment; on a fixed base the base's own station is used.
%   options.LaneIndices / PlanTransits are accepted for contract
%   compatibility and ignored (all segments are planned, transits always).
%   options.PlannerName selects the preset's tool (hairsim.planning.camClipperTool):
%   "camClipper" (feed direction ignored) or "camClipperAgainstFlow"
%   (registered separately; comb lifts only hair pushed against its flow).

arguments
    haircutState (1, 1) struct
    regionName (1, 1) string
    targetLengthM (1, 1) double {mustBeFinite, mustBeNonnegative}
    context (1, 1) struct
    options.OnProgress = []
    options.LaneIndices (1, :) double = []
    options.PlanTransits (1, 1) logical = true
    options.PlannerName (1, 1) string = "camClipper"
end

if ~isfield(context, 'numJoints') || isempty(context.numJoints)
    context.numJoints = numel(context.qHome);
end
if ~isfield(context, 'redundantJointIndices') || isempty(context.redundantJointIndices)
    context.redundantJointIndices = 1:context.numJoints;
end
context.qHome = context.qHome(:);

goalState = hairsim.hair.setUniformTargetLengthGoal(haircutState, targetLengthM);
tool = hairsim.planning.camClipperTool(options.PlannerName);
headQuery = hairsim.tool.createHeadDistanceQuery(context.headMeshWorld);
toolpathPlan = hairsim.cam.planRegionToolpaths(goalState, regionName, context.headMeshWorld, tool, ...
    'HeadQuery', headQuery, 'OnProgress', options.OnProgress);
[regionPlan, post] = hairsim.post.postProcessToolpaths(toolpathPlan, context, headQuery, goalState, ...
    'PlannerName', options.PlannerName, 'OnProgress', options.OnProgress);
comparison = hairsim.eval.compareIdealVsRobot(toolpathPlan, post);
regionPlan.targetLengthM = targetLengthM;
regionPlan.camClipper = struct('toolpathPlan', toolpathPlan, 'comparison', comparison, ...
    'pieces', rmfield(post.pieces, 'walk'), 'stationsUsedDeg', post.stationsUsedDeg);
end
