function toolpathPlan = planRegionToolpaths(haircutState, regionName, headMeshWorld, tool, options)
%PLANREGIONTOOLPATHS The CAM layer of the camClipper pipeline for one
%   region: machining surface -> raster passes -> segments (CL data) ->
%   rest machining + edge cleanup -> ideal coverage C_A. Robot-free: no IK,
%   no robot collision -- that is hairsim.post.postProcessToolpaths' job.
%
%   haircutState must carry a length goal. Returns toolpathPlan (the
%   "tool-position file", saveable with save()):
%     region, tool, surface (headQuery stripped), segments (struct array,
%     hairsim.cam.buildSegments), coveredMain / covered (region roots
%     covered by the main raster / by everything), C_main, C_A (over roots
%     needing cut), placement (hairsim.eval.buildPlacementMap, if
%     ComputePlacement), restHistory, passInfo, blockedFeetWorldM, timing,
%     flowSplit (hairsim.cam.splitSurfaceByFlow info).
%   A region whose hair flow diverges (the crown) is rastered as two parts,
%   each against its own flow; options.FlowSplitMinResultant = 0 disables.

arguments
    haircutState (1, 1) struct
    regionName (1, 1) string
    headMeshWorld (1, 1) struct
    tool (1, 1) struct
    options.HeadQuery = []
    options.ComputePlacement (1, 1) logical = true
    options.RestMachining (1, 1) logical = true
    options.OverlapFraction (1, 1) double = 0.3
    % piece length of the main raster (hairsim.cam.buildSegments); Inf =
    % one segment per whole pass (the "do not split lanes" ablation)
    options.SegmentLengthM (1, 1) double {mustBePositive} = 0.04
    options.FlowSplitMinResultant (1, 1) double = 0.5
    options.OnProgress = []
end

timer = tic;
report = @(msg) progress(options.OnProgress, msg);
headQuery = options.HeadQuery;
if isempty(headQuery)
    headQuery = hairsim.tool.createHeadDistanceQuery(headMeshWorld);
end

report("CAM " + regionName + ": machining surface");
surface = hairsim.cam.buildMachiningSurface(haircutState, regionName, headMeshWorld, headQuery);

report("CAM " + regionName + ": raster passes");
[parts, flowSplit] = hairsim.cam.splitSurfaceByFlow(surface, 'MinResultant', options.FlowSplitMinResultant);
if flowSplit.split
    report(sprintf("CAM %s: flow diverges (resultant %.2f), raster in 2 parts of %d/%d roots", ...
        regionName, flowSplit.resultant, flowSplit.partSizes));
end
passes = {};
passInfo = struct('planeOffsetsM', {}, 'stepOverM', {}, 'targetStepM', {});
for k = 1:numel(parts)
    [partPasses, passInfo(k)] = hairsim.cam.generateRasterPasses(parts{k}, headMeshWorld, tool, ...
        'OverlapFraction', options.OverlapFraction);
    passes = [passes, partPasses]; %#ok<AGROW>
end
[segments, blocked] = hairsim.cam.buildSegments(passes, surface, tool, "main", ...
    'SegmentLengthM', options.SegmentLengthM);
timing.mainS = toc(timer);

covered = false(size(surface.rootsWorldM, 1), 1);
for s = 1:numel(segments)
    covered = covered | hairsim.eval.sweepCombLiftCoverage(surface.rootsWorldM, segments(s).T_cut, tool, ...
        'FlowDirs', surface.flowDirs);
end
coveredMain = covered;

restHistory = [];
if options.RestMachining
    report("CAM " + regionName + ": rest machining");
    [segments, covered, restHistory] = hairsim.cam.restMachining(surface, segments, covered, tool, ...
        'OnProgress', options.OnProgress);
end
timing.restS = toc(timer) - timing.mainS;

placement = [];
if options.ComputePlacement
    report("CAM " + regionName + ": placement map");
    placement = hairsim.eval.buildPlacementMap(surface, tool, 'OnProgress', options.OnProgress);
end
timing.totalS = toc(timer);

needs = surface.needsCut;
toolpathPlan = struct();
toolpathPlan.region = regionName;
toolpathPlan.tool = tool;
toolpathPlan.surface = rmfield(surface, 'headQuery');
toolpathPlan.segments = segments;
toolpathPlan.coveredMain = coveredMain;
toolpathPlan.covered = covered;
toolpathPlan.numNeedsCut = nnz(needs);
toolpathPlan.C_main = mean(coveredMain(needs));
toolpathPlan.C_A = mean(covered(needs));
toolpathPlan.placement = placement;
toolpathPlan.restHistory = restHistory;
toolpathPlan.passInfo = passInfo;
toolpathPlan.blockedFeetWorldM = blocked;
toolpathPlan.timing = timing;
toolpathPlan.flowSplit = flowSplit;
report(sprintf("CAM %s: %d segments, C_main %.1f%%, C_A %.1f%% (%.0f s)", regionName, ...
    numel(segments), 100 * toolpathPlan.C_main, 100 * toolpathPlan.C_A, timing.totalS));
end

function progress(fn, msg)
if ~isempty(fn)
    fn(char(msg));
end
end
