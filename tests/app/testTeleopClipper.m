function result = testTeleopClipper(options)
%TESTTELEOPCLIPPER Headless clipper-Teleoperation session (no figure):
%   from the clipper-safe Home, lower onto the scalp (F), turn the feed
%   against the flow (G), push forward with Space held (W), pull back with
%   Space held (S), keep pressing F. Every tick goes through the same
%   stepTeleop + applyPoseTick the GUI uses. Checks:
%     - every accepted pose keeps the clipper >= tool.safetyClearanceM off
%       the exact head mesh (also while F is held at the lowest standoff);
%     - G leaves the feed within 10 deg of against-the-flow;
%     - the forward pass cuts something, and the strands cut are EXACTLY
%       the ones the comb-lift predicate (hairsim.eval.sweepCombLiftCoverage)
%       gives for the recorded cut ticks -- Teleop cuts like Auto;
%     - pulling back with Space held cuts nothing;
%     - pushing WITH the hair flow (feed turned round on uncut hair) cuts
%       nothing, where the same poses would cut without the 45 deg rule;
%     - idle frames between key presses (hairsim.app.stepIdleHair) keep
%       the hair moving without breaking a cut sweep, and afterwards the
%       clippings fall to the floor instead of hanging in the air.

arguments
    options.AdapterName (1, 1) string = "ur5e"
    options.BasePlacementId (1, 1) string = "fixed"
    options.ForwardSteps (1, 1) double {mustBeInteger, mustBePositive} = 12
    % idle frames between key presses, as in the GUI (the hair keeps moving
    % while the arm stands still; this must not break a cut sweep)
    options.IdleFramesBetweenKeys (1, 1) double {mustBeInteger, mustBeNonnegative} = 3
end

scene = hairsim.api.loadScene();
robotAppState = hairsim.app.robot.createRobotAppState(options.AdapterName, scene);
if options.BasePlacementId ~= "fixed"
    robotAppState = hairsim.app.robot.switchBase(robotAppState, options.BasePlacementId);
end
hairAppState = hairsim.app.hair.createHairAppState(scene);
teleopState = hairsim.app.teleop.createTeleopState(scene);
[hairAppState, flowDirs] = hairsim.app.hair.installClipperCut(hairAppState, scene.headScene.headMeshWorld, ...
    "camClipperAgainstFlow", 0.005);
teleopState = hairsim.app.teleop.setHair(teleopState, hairAppState.hairLayer.haircutState, flowDirs);
qHome = hairsim.app.robot.clipperSafeHome(robotAppState, scene, []);
robotAppState.state.qCurrent = qHome;
robotAppState.state.qCommanded = qHome;
teleopState = hairsim.app.teleop.syncToRobot(teleopState, robotAppState);
tool = teleopState.tool;
floorZ = scene.headScene.headMeshWorld.bboxMin(3) - 0.35 * 0.5 * max(scene.headScene.headMeshWorld.bboxSize);
countsBefore = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;
hairs0 = hairAppState.hairLayer.haircutState.hairState.hairs;
target = hairAppState.hairLayer.haircutState.lengthGoal.targetActiveNodeCounts;

ticks = struct('phase', {}, 'solved', {}, 'cutFlag', {}, 'status', {}, 'T_tcp', {}, 'clearanceM', {}, 'counts', {});
phase = "";

phase = "lower";
press(repmat("f", 1, 8));
fprintf('after lowering: %s, height %.1f mm, touching %d\n', teleopState.hud.status, ...
    teleopState.hud.standoffMm, teleopState.hud.inContact);
phase = "snap";
press("g");
fprintf('after G: feed %.1f deg off against-flow\n', teleopState.hud.feedAngleDeg);
assert(teleopState.hud.feedAngleDeg <= 10, "G left the feed %.1f deg off against-flow", teleopState.hud.feedAngleDeg);

phase = "forward";
teleopState = hairsim.app.teleop.handleKey(teleopState, 'space', true);
press(repmat("w", 1, options.ForwardSteps));
phase = "back";
press(repmat("s", 1, 4));
phase = "slide";                % onto uncut hair, then turn round: feed WITH the flow
press([repmat("d", 1, 6), repmat("q", 1, 12)]);
phase = "withFlow";
press(repmat("w", 1, 6));
teleopState = hairsim.app.teleop.handleKey(teleopState, 'space', false);
phase = "pressDown";
press(repmat("f", 1, 5));

% ---- checks
accepted = ticks([ticks.solved]);
clearances = [accepted.clearanceM];
refusals = unique([strings(1, 0), ticks(~[ticks.solved]).status]);
fprintf('%d ticks (%d accepted, %d refused: %s)\n', numel(ticks), numel(accepted), ...
    nnz(~[ticks.solved]), strjoin(refusals, ", "));
fprintf('min clipper clearance over accepted poses: %.2f mm\n', 1e3 * min(clearances));
assert(all(clearances >= tool.safetyClearanceM - 1e-9), "an accepted pose is closer than the safety clearance");

counts = hairAppState.hairLayer.haircutState.hairState.activeNodeCounts;
cutStrands = find(counts < countsBefore);
backTicks = ticks([ticks.phase] == "back");
assert(isequal(backTicks(1).counts, backTicks(end).counts) && ~any([backTicks.cutFlag]), ...
    "pulling back with Space held cut hair");
assert(~isempty(cutStrands), "the forward pass cut nothing (is the clipper on hair?)");
assert(all(counts(cutStrands) == target(cutStrands)), "a strand was cut somewhere other than its target");

% pushing WITH the flow lifts nothing, although the same poses would cut
% if the feed direction were ignored (so the check is not vacuous)
withFlow = ticks([ticks.phase] == "withFlow");
assert(isequal(withFlow(1).counts, withFlow(end).counts), "pushing with the hair flow cut hair");
before = ticks(find([ticks.phase] == "withFlow", 1) - 1).counts;
open = find(before > target);
openRoots = [hairs0.X(1, open)', hairs0.Y(1, open)', hairs0.Z(1, open)'];
noCheck = tool;
noCheck.maxFeedAngleRad = Inf;
wouldCut = hairsim.eval.sweepCombLiftCoverage(openRoots, cat(3, withFlow([withFlow.cutFlag]).T_tcp), noCheck);
fprintf('with-flow pass: cut 0 strands; without the feed check it would cut %d\n', nnz(wouldCut));
assert(nnz(wouldCut) > 0, "with-flow check is vacuous: the pass is not over uncut hair");

% the comb-lift predicate over each run of consecutive cut ticks
needs = find(countsBefore > target);
hairs = hairAppState.hairLayer.haircutState.hairState.hairs;
roots = [hairs.X(1, needs)', hairs.Y(1, needs)', hairs.Z(1, needs)'];
covered = false(numel(needs), 1);
cutFlags = [ticks.cutFlag];
edges = diff([false, cutFlags, false]);
starts = find(edges == 1);
stops = find(edges == -1) - 1;
for r = 1:numel(starts)
    T = cat(3, ticks(starts(r):stops(r)).T_tcp);
    covered = covered | hairsim.eval.sweepCombLiftCoverage(roots, T, tool, 'FlowDirs', flowDirs(needs, :));
end
predicted = needs(covered);
fprintf('cut %d strands in %d cut ticks; predicate predicts %d; identical: %d\n', numel(cutStrands), ...
    nnz(cutFlags), numel(predicted), isequal(sort(cutStrands(:)), sort(predicted(:))));
assert(isequal(sort(cutStrands(:)), sort(predicted(:))), "Teleop cut differs from the comb-lift predicate");
% clippings fall while the arm stands still (they used to freeze mid-air)
numDebris = numel(hairAppState.hairLayer.haircutState.debrisList);
for idle = 1:3000
    if isempty(hairAppState.hairLayer.haircutState.debrisList)
        break;
    end
    hairAppState = hairsim.app.stepIdleHair(hairAppState, robotAppState, [], scene, floorZ);
end
fprintf('clippings in the air after the session: %d; after %d idle frames: %d\n', numDebris, idle - 1, ...
    numel(hairAppState.hairLayer.haircutState.debrisList));
assert(isempty(hairAppState.hairLayer.haircutState.debrisList), "clippings still in the air after idle frames");
assert(isequal(hairAppState.hairLayer.haircutState.hairState.activeNodeCounts, counts), "an idle frame cut hair");
fprintf('PASS: clipper Teleop (%s, %s)\n', options.AdapterName, options.BasePlacementId);
result = struct('ticks', ticks, 'cutStrands', cutStrands, 'minClearanceM', min(clearances));

    function press(keys)
        % one key press + release, then tick until Teleop has nothing more to do
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
                mc = robotAppState.modelContext;
                T = getTransform(mc.robot, robotAppState.state.qCurrent, char(mc.endEffector)) * tool.T_ee_tcp;
                c = hairsim.tool.evaluateClipperClearance(T, tool, teleopState.headQuery, 'CapM', 0.2);
                ticks(end + 1) = struct('phase', phase, 'solved', poseCommand.solved, ...
                    'cutFlag', poseCommand.cutFlag, 'status', string(poseCommand.statusLabel), ...
                    'T_tcp', T, 'clearanceM', c.minM, ...
                    'counts', hairAppState.hairLayer.haircutState.hairState.activeNodeCounts); %#ok<AGROW>
            end
            for idle = 1:options.IdleFramesBetweenKeys
                hairAppState = hairsim.app.stepIdleHair(hairAppState, robotAppState, [], scene, floorZ);
            end
        end
    end
end
