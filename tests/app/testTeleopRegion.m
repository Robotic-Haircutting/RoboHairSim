function result = testTeleopRegion(options)
%TESTTELEOPREGION Headless clipper Teleoperation confined to one region
%   (hairsim.app.teleop.setRegion + hairsim.app.hair.installClipperCut
%   'Region'). From the clipper-safe Home, the region is the one under the
%   clipper; lower (F), turn against the flow (G), then with Space held
%   push forward and slide both ways far enough to reach the boundary.
%   Checks:
%     - the session hits the boundary (some move is REJECTED:OUTSIDE_REGION);
%     - every accepted foot lies in the region (regionMembership);
%     - something is cut, and only strands of the region are cut;
%     - switched to another region the clipper is outside it, and every
%       accepted move then brings it closer (it can be driven in, not away);
%     - the highlight covers exactly the head faces regionMembership puts
%       in the region;
%     - "Show uncut" marks exactly the region strands still above target.

arguments
    options.AdapterName (1, 1) string = "ur5e"
    options.Steps (1, 1) double {mustBeInteger, mustBePositive} = 25
end

scene = hairsim.api.loadScene();
robotAppState = hairsim.app.robot.createRobotAppState(options.AdapterName, scene);
hairAppState = hairsim.app.hair.createHairAppState(scene);
teleopState = hairsim.app.teleop.createTeleopState(scene);
qHome = hairsim.app.robot.clipperSafeHome(robotAppState, scene, []);
robotAppState.state.qCurrent = qHome;
robotAppState.state.qCommanded = qHome;
installCut("");
teleopState = hairsim.app.teleop.syncToRobot(teleopState, robotAppState);
floorZ = scene.headScene.headMeshWorld.bboxMin(3) - 0.35 * 0.5 * max(scene.headScene.headMeshWorld.bboxSize);

% the region under the clipper
[~, k] = min(vecnorm(teleopState.hairRoots - teleopState.surface.footWorldM, 2, 2));
region = teleopState.strandRegions(k);
installCut(region);
fprintf('region under the clipper: %s (reach %.1f mm)\n', region, 1000 * teleopState.regionReachM);
assert(teleopState.hud.inRegion, "the clipper is not inside the region under it");
regions = teleopState.strandRegions;
countsBefore = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;

ticks = struct('solved', {}, 'status', {}, 'foot', {});
press(repmat("f", 1, 8));
press("g");
teleopState = hairsim.app.teleop.handleKey(teleopState, 'space', true);
press([repmat("w", 1, options.Steps), repmat("d", 1, options.Steps), repmat("a", 1, 2 * options.Steps), ...
    repmat("w", 1, options.Steps)]);
teleopState = hairsim.app.teleop.handleKey(teleopState, 'space', false);

statuses = [ticks.status];
accepted = ticks([ticks.solved]);
fprintf('%d ticks, %d accepted; refusals: %s\n', numel(ticks), numel(accepted), ...
    strjoin(unique([strings(1, 0), statuses(~[ticks.solved])]), ", "));
assert(any(statuses == "REJECTED:OUTSIDE_REGION"), "never reached the region boundary");
assert(all(hairsim.app.teleop.regionMembership(teleopState, vertcat(accepted.foot))), ...
    "an accepted foot is outside the region");
cut = find(hairAppState.hairLayer.haircutState.hairState.activeNodeCounts < countsBefore);
fprintf('cut %d strands, regions: %s\n', numel(cut), strjoin(unique(regions(cut))', ", "));
assert(~isempty(cut), "nothing was cut");
assert(all(regions(cut) == region), "a strand outside the region was cut");

% "Show uncut": exactly the region strands still above target, at their roots
hc = hairAppState.hairLayer.haircutState;
uncut = hairsim.app.hair.uncutStrands(hc);
expectedLeft = find(regions' == region & hc.hairState.activeNodeCounts > hc.lengthGoal.targetActiveNodeCounts);
assert(isequal(uncut.idx, expectedLeft) && uncut.numNeeds == nnz(regions == region) && ...
    uncut.numDone == numel(cut) && ~any(ismember(cut, uncut.idx)), "uncut strands are not the region strands left");
fig = figure('Visible', 'off');
h = hairsim.app.view.drawUncutStrands(axes(fig), uncut);
assert(numel(h.XData) == numel(uncut.idx), "not one red dot per uncut strand");
fprintf('show uncut: %d / %d done, %d red dots\n', uncut.numDone, uncut.numNeeds, numel(h.XData));
close(fig);

% another region: the clipper is outside; only moves toward it are taken
others = setdiff(unique(regions), region);
gaps = arrayfun(@(r) min(vecnorm(teleopState.hairRoots(regions == r, :) - ...
    teleopState.surface.footWorldM, 2, 2)), others);
[~, order] = sort(gaps);
other = others(order(2));         % not the adjacent one: the clipper has a way to go
installCut(other);
assert(~teleopState.hud.inRegion, "the clipper is inside a region it was confined out of");
ticks = ticks([]);
startFoot = teleopState.surface.footWorldM;
press([repmat("d", 1, options.Steps), repmat("a", 1, 2 * options.Steps), repmat("w", 1, options.Steps), ...
    repmat("s", 1, 2 * options.Steps)]);
feet = [startFoot; vertcat(ticks.foot)];
[inside, distanceM] = hairsim.app.teleop.regionMembership(teleopState, feet);
moved = vecnorm(diff(feet), 2, 2) > 0;          % a foot move (not a settling / turning tick)
movedOutside = moved & ~inside(1:end - 1) & ~inside(2:end);
dropped = diff(distanceM) < 0;
inside = inside(2:end);
fprintf('%s (%.0f mm away): %d accepted moves while outside, %d of them not closer; ended inside: %d\n', ...
    other, 1000 * gaps(order(2)), nnz(movedOutside), nnz(movedOutside & ~dropped), inside(end));
assert(nnz(movedOutside) > 0, "approach check is vacuous: no move while outside the region");
assert(~any(movedOutside & ~dropped), "an accepted move outside the region did not approach it");
assert(any([ticks.status] == "REJECTED:OUTSIDE_REGION"), "no move away from the region was refused");

% the highlight = the faces regionMembership puts in the region
fig = figure('Visible', 'off');
cleanup = onCleanup(@() close(fig));
ax = axes(fig);
h = hairsim.app.view.drawTeleopRegion(ax, scene.headScene.headMeshWorld, teleopState);
F = double(scene.headScene.headMeshWorld.faces);
V = double(scene.headScene.headMeshWorld.vertices);
expected = hairsim.app.teleop.regionMembership(teleopState, (V(F(:, 1), :) + V(F(:, 2), :) + V(F(:, 3), :)) / 3);
assert(isequal(h.Faces, F(expected, :)), "the highlight is not the region");
teleopState = hairsim.app.teleop.setRegion(teleopState, "");
assert(isempty(hairsim.app.view.drawTeleopRegion(ax, scene.headScene.headMeshWorld, teleopState)) && ...
    isempty(findobj(ax, 'Tag', 'HairSimTeleopRegion')), "the highlight stayed with no region selected");
fprintf('highlight: %d of %d head faces\n', nnz(expected), numel(expected));
fprintf('PASS: Teleop region limit (%s, %s)\n', options.AdapterName, region);
result = struct('region', region, 'cutStrands', cut);

    function installCut(name)
        [hairAppState, flowDirs] = hairsim.app.hair.installClipperCut(hairAppState, ...
            scene.headScene.headMeshWorld, "camClipperAgainstFlow", 0.005, 'Region', name);
        teleopState = hairsim.app.teleop.setHair(teleopState, hairAppState.hairLayer.haircutState, flowDirs);
        teleopState = hairsim.app.teleop.setRegion(teleopState, name);
    end

    function press(keys)
        for key = keys
            teleopState = hairsim.app.teleop.handleKey(teleopState, key, true);
            teleopState = hairsim.app.teleop.handleKey(teleopState, key, false);
            for guard = 1:50
                [teleopState, poseCommand] = hairsim.app.teleop.stepTeleop(teleopState, robotAppState, true);
                if isempty(poseCommand)
                    break;
                end
                [robotAppState, hairAppState] = hairsim.app.applyPoseTick(poseCommand, robotAppState, ...
                    hairAppState, struct(), [], scene, floorZ, 'Record', false);
                ticks(end + 1) = struct('solved', poseCommand.solved, ...
                    'status', string(poseCommand.statusLabel), 'foot', teleopState.surface.footWorldM); %#ok<AGROW>
            end
        end
    end
end
