function testCamClipperFoundations(options)
%TESTCAMCLIPPERFOUNDATIONS Step 0 + lines A/B of the camClipper plan:
%   clipper tool model, exact signed head distance, clipper clearance,
%   comb-lift coverage predicate, comb-lift cut, per-region length goal.
%   Synthetic geometry first (analytic answers), then the real head.
%   Errors on the first failure; prints PASS lines otherwise.

arguments
    options.DataDir (1, 1) string = ""            % "" = hairsim.api.loadScene's default
    options.PlanningDataDir (1, 1) string = ""
end

tool = hairsim.tool.createClipperTool();

% ---------- 1. signed distance on an analytic sphere (r = 0.1 m)
[sx, sy, sz] = sphere(80);
fv = surf2patch(0.1 * sx, 0.1 * sy, 0.1 * sz, 'triangles');
fv = dropDegenerate(fv);
V = fv.vertices;
F = fv.faces;
c = (V(F(:, 1), :) + V(F(:, 2), :) + V(F(:, 3), :)) / 3;
nrm = cross(V(F(:, 2), :) - V(F(:, 1), :), V(F(:, 3), :) - V(F(:, 1), :), 2);
if sum(dot(c, nrm, 2)) < 0
    F = F(:, [1, 3, 2]);
end
sphereMesh = struct('faces', F, 'vertices', V);
q = hairsim.tool.createHeadDistanceQuery(sphereMesh);
pts = [0.12, 0, 0; 0, 0.09, 0; 0, 0, 0.1005; 0.07, 0.07, 0.02];
d = hairsim.tool.signedDistanceToHead(pts, q);
expected = vecnorm(pts, 2, 2) - 0.1;
check(max(abs(d - expected)) < 6e-4, sprintf('signed distance on sphere (max err %.2g m)', max(abs(d - expected))));
dCap = hairsim.tool.signedDistanceToHead([0.3, 0, 0], q, 0.03);
check(dCap == 0.03, 'distance cap returns +cap for far points');

% ---------- 2. coverage predicate on a flat scalp (plane z = 0, n = +z)
[gx, gy] = meshgrid(-0.05:0.002:0.05, -0.03:0.001:0.03);
roots = [gx(:), gy(:), zeros(numel(gx), 1)];
T = poseSequence([-0.02, 0, 0], [0.02, 0, 0], tool, 0.002, 0, 0, 9);
covered = hairsim.eval.sweepCombLiftCoverage(roots, T, tool);
inBand = abs(roots(:, 2)) <= tool.bladeHalfWidthM & ...
    roots(:, 1) >= -0.02 + tool.footprintXRangeM(1) - 1e-9 & roots(:, 1) <= 0.02 + tool.footprintXRangeM(2) + 1e-9;
check(isequal(covered, inBand), 'flat pass covers exactly the 26 mm band over the swept length');
Thigh = poseSequence([-0.02, 0, 0], [0.02, 0, 0], tool, tool.maxLocalHeightM + 0.001, 0, 0, 9);
check(~any(hairsim.eval.sweepCombLiftCoverage(roots, Thigh, tool)), 'blade above the contact tolerance cuts nothing');
Tsparse = poseSequence([-0.02, 0, 0], [0.02, 0, 0], tool, 0.002, 0, 0, 2);
check(isequal(hairsim.eval.sweepCombLiftCoverage(roots, Tsparse, tool), covered), ...
    'two far-apart poses are swept, not sampled');
roll = deg2rad(30);
% rolled blade lifted so its LOW edge sits at the nominal stand-off (what
% resolveContactStandoff does on a real head): only the low side is
% within the comb tolerance
Troll = poseSequence([-0.02, 0, 0], [0.02, 0, 0], tool, ...
    tool.nominalStandoffM + tool.bladeHalfWidthM * sin(roll), 0, roll, 9);
cRoll = hairsim.eval.sweepCombLiftCoverage(roots, Troll, tool);
check(nnz(cRoll) > 0 && nnz(cRoll) < nnz(covered), sprintf('30 deg roll cuts with one edge only (%d of %d roots)', nnz(cRoll), nnz(covered)));
yHit = roots(cRoll, 2);
check(all(yHit > 0) || all(yHit < 0), 'rolled blade covers one side of the centre line');
% feed-direction check: the pass feeds +x
toolDir = hairsim.tool.createClipperTool('MaxFeedAngleRad', deg2rad(45));
nR = size(roots, 1);
check(isequal(hairsim.eval.sweepCombLiftCoverage(roots, T, toolDir, 'FlowDirs', repmat([-1, 0, 0], nR, 1)), covered), ...
    'feed against the flow: direction check keeps the full band');
check(~any(hairsim.eval.sweepCombLiftCoverage(roots, T, toolDir, 'FlowDirs', repmat([1, 0, 0], nR, 1))), ...
    'feed with the flow lifts nothing');
check(~any(hairsim.eval.sweepCombLiftCoverage(roots, T, toolDir, 'FlowDirs', repmat([0, 1, 0], nR, 1))), ...
    'feed across the flow lifts nothing at 45 deg');
check(isequal(hairsim.eval.sweepCombLiftCoverage(roots, T, toolDir, 'FlowDirs', repmat([-cosd(40), sind(40), 0], nR, 1)), covered), ...
    'feed 40 deg off against-flow still lifts at 45 deg');
threw = false;
try
    hairsim.eval.sweepCombLiftCoverage(roots, T, toolDir);
catch err
    threw = err.identifier == "hairsim:eval:FeedCheckNeedsFlow";
end
check(threw, 'direction check without FlowDirs is an error, not a silent pass');

% ---------- 3. real head: clearance, lift, comb-lift cut, region goal
sceneArgs = {};
if strlength(options.DataDir) > 0
    sceneArgs = [sceneArgs, {'DataDir', options.DataDir}];
end
if strlength(options.PlanningDataDir) > 0
    sceneArgs = [sceneArgs, {'PlanningDataDir', options.PlanningDataDir}];
end
scene = hairsim.api.loadScene(sceneArgs{:});
head = scene.headScene.headMeshWorld;
headQuery = hairsim.tool.createHeadDistanceQuery(head);
hs = scene.haircutState;
rootsAll = [hs.hairState.hairs.X(1, :)', hs.hairState.hairs.Y(1, :)', hs.hairState.hairs.Z(1, :)'];
[~, top] = max(rootsAll(:, 3));
foot = rootsAll(top, :);
[~, footOnMesh, ~] = hairsim.tool.signedDistanceToHead(foot, headQuery);
nOut = (foot - mean(double(head.vertices), 1));
nOut = nOut / norm(nOut);
Tn = hairsim.tool.buildTcpPose(footOnMesh, nOut, [1, 0, 0], tool);
clr = hairsim.tool.evaluateClipperClearance(Tn, tool, headQuery);
check(clr.contactMinM > 0 && clr.contactMinM < 0.003, sprintf('nominal pose on the crown: contact clearance %.2f mm', 1e3 * clr.contactMinM));
% 2 mm into the scalp: a lift within the 3 mm comb tolerance clears it
Tin = Tn;
Tin(1:3, 4) = Tin(1:3, 4) - 0.002 * nOut';
[Tlift, okLift, liftM] = hairsim.tool.resolveContactStandoff(Tin, nOut, tool, headQuery);
cl2 = hairsim.tool.evaluateClipperClearance(Tlift, tool, headQuery);
check(okLift && cl2.ok && liftM <= tool.contactToleranceM, ...
    sprintf('resolveContactStandoff lifts a pose 2 mm in to a clear one (lift %.2f mm)', 1e3 * liftM));
% 4 mm in: the full tolerance is not enough -- reported, not hidden
Tin(1:3, 4) = Tn(1:3, 4) - 0.004 * nOut';
[~, okDeep, liftDeep] = hairsim.tool.resolveContactStandoff(Tin, nOut, tool, headQuery);
check(~okDeep && abs(liftDeep - tool.contactToleranceM) < 1e-12, ...
    sprintf('resolveContactStandoff refuses a pose 4 mm in (lift capped at %.2f mm)', 1e3 * liftDeep));

goal = hairsim.hair.setUniformTargetLengthGoal(hs, 0.005);
reg = hairsim.hair.setRegionTargetLengthGoal(hs, struct(), 0.005);
check(isequal(goal.lengthGoal.targetActiveNodeCounts, reg.lengthGoal.targetActiveNodeCounts), ...
    'region goal with no overrides == uniform goal');
mixed = hairsim.hair.setRegionTargetLengthGoal(hs, struct('scalp_back', 0), 0.005);
regions = hairsim.hair.getStrandRegions(hs);
zeroGoal = hairsim.hair.setUniformTargetLengthGoal(hs, 0);
back = regions == "scalp_back";
check(isequal(mixed.lengthGoal.targetActiveNodeCounts(back), zeroGoal.lengthGoal.targetActiveNodeCounts(back)) && ...
    isequal(mixed.lengthGoal.targetActiveNodeCounts(~back), goal.lengthGoal.targetActiveNodeCounts(~back)), ...
    'per-region override applies to that region only');

cutModel = hairsim.hair.cutting.createCombLiftCutModel(tool);
T_ee = Tn / tool.T_ee_tcp;
[hs2, info] = hairsim.hair.cutting.applyClipperHaircutStep(goal, T_ee, cutModel);
cutStrands = info.strandIndices;
check(info.cutCount > 0 && all(hs2.hairState.activeNodeCounts(cutStrands) == ...
    goal.lengthGoal.targetActiveNodeCounts(cutStrands)), sprintf('comb-lift cut on the crown: %d strands cut exactly to target', info.cutCount));
untouched = setdiff(1:numel(regions), cutStrands);
check(isequal(hs2.hairState.activeNodeCounts(untouched), goal.hairState.activeNodeCounts(untouched)), ...
    'strands outside the footprint are untouched');
fprintf('ALL FOUNDATION TESTS PASSED\n');
end

function T = poseSequence(p0, p1, tool, standoff, lateral, roll, numPoses)
T = zeros(4, 4, numPoses);
for k = 1:numPoses
    a = (k - 1) / max(numPoses - 1, 1);
    foot = (1 - a) * p0 + a * p1;
    T(:, :, k) = hairsim.tool.buildTcpPose(foot, [0, 0, 1], [1, 0, 0], tool, ...
        'StandoffM', standoff, 'LateralOffsetM', lateral, 'RollRad', roll);
end
end

function fv = dropDegenerate(fv)
V = fv.vertices;
F = fv.faces;
a = cross(V(F(:, 2), :) - V(F(:, 1), :), V(F(:, 3), :) - V(F(:, 1), :), 2);
fv.faces = F(vecnorm(a, 2, 2) > 1e-12, :);
end

function check(condition, label)
if ~condition
    error("camClipper:TestFailed", "FAIL: %s", label);
end
fprintf('PASS: %s\n', label);
end
