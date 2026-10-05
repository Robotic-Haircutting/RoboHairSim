function cmp = compareIdealVsRobot(toolpathPlan, post)
%COMPAREIDEALVSROBOT Evaluation A vs B of one region (same toolpath, same
%   static hair, same comb-lift predicate; only the pose source differs).
%     A = the CAM poses as planned (toolpathPlan.covered)
%     B = the FK poses of the robot strokes that are in the transit chain
%   Three-layer loss over the roots that need cutting:
%     1 - C_place        the clipper cannot be placed at all (tool limit)
%     C_place - C_A      placeable but the toolpath did not reach it
%     C_A - C_B          the robot arm (IK / collision / transit / tracking)
%   Each root lost between A and B is charged to the first CAM segment that
%   covered it in A and that segment's fate:
%     notSolved / robotHead / jointStep / clipperClearance / selfCollision
%                        the piece of the segment covering it was not walkable
%                        (dominant rejection at the station that got furthest)
%     transit            walkable, but no transit reached the stroke
%     tracking           executed, yet the achieved poses missed the root
%   cmp.lossBreakdown sums to (C_A - C_B) * numNeedsCut roots.

needs = toolpathPlan.surface.needsCut;
roots = toolpathPlan.surface.rootsWorldM;
tool = toolpathPlan.tool;
numNeeds = nnz(needs);

C_place = nan;
if ~isempty(toolpathPlan.placement)
    C_place = toolpathPlan.placement.C_place;
end
lostIdx = find(needs & toolpathPlan.covered & ~post.coveredChain);
reasons = strings(numel(lostIdx), 1);
segs = toolpathPlan.segments;
for r = 1:numel(lostIdx)
    root = roots(lostIdx(r), :);
    reasons(r) = "unattributed";
    for s = 1:numel(segs)
        [hit, firstPose] = hairsim.eval.sweepCombLiftCoverage(root, segs(s).T_cut, tool, ...
            'FlowDirs', toolpathPlan.surface.flowDirs(lostIdx(r), :));
        if ~hit
            continue;
        end
        reasons(r) = pieceFate(post, s, firstPose);
        break;
    end
end
cats = ["notSolved", "robotHead", "jointStep", "clipperClearance", "selfCollision", ...
    "transit", "tracking", "unattributed"];
counts = arrayfun(@(c) nnz(reasons == c), cats);
cmp = struct();
cmp.region = toolpathPlan.region;
cmp.numNeedsCut = numNeeds;
cmp.C_place = C_place;
cmp.C_A = mean(toolpathPlan.covered(needs));
cmp.C_exec = post.C_exec;
cmp.C_B = post.C_B;
cmp.deltaC = cmp.C_A - cmp.C_B;
cmp.lossBreakdown = cell2struct(num2cell(counts(:)), cellstr(cats(:)), 1);
cmp.lostRootIdx = lostIdx;
cmp.lostReason = reasons;
end

function fate = pieceFate(post, segIdx, cutSample)
% the piece of segment segIdx whose CUT range contains cutSample (pieces of
% a split segment overlap by one sample; the executable one wins)
fate = "tracking";
pieces = post.pieces([post.pieces.segIdx] == segIdx);
inRange = arrayfun(@(p) cutSample >= p.cutRange(1) && cutSample <= p.cutRange(2), pieces);
cand = pieces(inRange);
if isempty(cand)
    fate = "unattributed";
    return;
end
okCand = cand([cand.ok]);
if isempty(okCand)
    fate = cand(1).reason;
    return;
end
% executable: lost only if none of its executable pieces made the chain
if any(arrayfun(@(p) any(post.chain == p.strokeIdx), okCand))
    fate = "tracking";
else
    fate = "transit";
end
end
