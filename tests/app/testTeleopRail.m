function result = testTeleopRail(options)
%TESTTELEOPRAIL Headless Teleop rail slide ([ / ]) on the Rail setup
%   (rail180). From the clipper-safe Home, lower onto the scalp, then slide
%   the base both ways and keep driving. Checks:
%     - every accepted slide moves the station by exactly RailStepRad and
%       leaves the clipper where it was (TCP within the IK tolerance), and
%       the robot model is at the new station (FK of the new model at the
%       new q = the reported pose);
%     - every accepted pose keeps the clipper >= tool.safetyClearanceM off
%       the head;
%     - sliding on toward the end of the ring stops (an arm link meeting the
%       head is refused like any pose); past RailLimitsRad it is refused
%       (RAIL_LIMIT, tested with the end moved one step ahead of the base)
%       and the station stays inside the limits;
%     - after slides the clipper still drives on the scalp (W accepted);
%     - on the fixed base [ ] is refused (NO_RAIL) and nothing moves.

arguments
    options.AdapterName (1, 1) string = "ur5e"
end

scene = hairsim.api.loadScene();
robotAppState = hairsim.app.robot.createRobotAppState(options.AdapterName, scene);
robotAppState = hairsim.app.robot.switchBase(robotAppState, "rail180");
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
floorZ = scene.headScene.headMeshWorld.bboxMin(3) - 0.1;

ticks = struct('key', {}, 'solved', {}, 'status', {}, 'thetaBefore', {}, 'thetaAfter', {}, ...
    'tcpBefore', {}, 'tcpAfter', {}, 'reportedTcp', {}, 'clearanceM', {});
press(repmat("f", 1, 8));
press([repmat("rightbracket", 1, 4), repmat("leftbracket", 1, 8)]);
press(repmat("w", 1, 6));
press(repmat("rightbracket", 1, 30));          % on toward the end of the ring (an arm link may meet the head first)
% the end of the ring: back off two (reachable) steps, put the end one step
% ahead, press past it
press(repmat("leftbracket", 1, 2));
teleopState.railLimitsRad(2) = robotAppState.state.railThetaRad + teleopState.railStepRad;
press(repmat("rightbracket", 1, 3));

slides = ticks(ismember([ticks.key], ["leftbracket", "rightbracket"]) & [ticks.solved]);
stepDeg = rad2deg(abs([slides.thetaAfter] - [slides.thetaBefore]));
held = vecnorm(cell2mat({slides.tcpAfter}') - cell2mat({slides.tcpBefore}'), 2, 2);
modelErr = vecnorm(cell2mat({slides.tcpAfter}') - cell2mat({slides.reportedTcp}'), 2, 2);
fprintf('%d accepted slides, station steps %s deg, clipper moved at most %.2f mm, model error %.2g mm\n', ...
    numel(slides), mat2str(unique(round(stepDeg, 6))'), 1000 * max(held), 1000 * max(modelErr));
assert(numel(slides) >= 4, "fewer than 4 slides accepted");
assert(all(abs(stepDeg - rad2deg(teleopState.railStepRad)) < 1e-6), "a slide did not move one rail step");
assert(all(held <= teleopState.params.maxToolTipErrorM + 1e-9), "a slide moved the clipper");
assert(all(modelErr < 1e-9), "robot model is not at the slid station");
accepted = ticks([ticks.solved]);
fprintf('min clipper clearance over accepted poses: %.2f mm\n', 1000 * min([accepted.clearanceM]));
assert(all([accepted.clearanceM] >= tool.safetyClearanceM - 1e-9), "an accepted pose is closer than the safety clearance");
statuses = [ticks.status];
refusals = unique([strings(1, 0), statuses(~[ticks.solved])]);
fprintf('refusals: %s; final station %.0f deg\n', strjoin(refusals, ", "), rad2deg(robotAppState.state.railThetaRad));
assert(any(statuses == "REJECTED:RAIL_LIMIT"), "sliding past the end of the ring was not refused");
assert(robotAppState.state.railThetaRad <= teleopState.railLimitsRad(2) + 1e-9 && ...
    robotAppState.state.railThetaRad >= teleopState.railLimitsRad(1) - 1e-9, "station outside the rail limits");
drive = ticks([ticks.key] == "w");
assert(any([drive.solved]), "the clipper no longer drives after sliding the rail");

% fixed base: no rail
robotAppState = hairsim.app.robot.switchBase(robotAppState, "fixed");
qHome = hairsim.app.robot.clipperSafeHome(robotAppState, scene, []);
robotAppState.state.qCurrent = qHome;
robotAppState.state.qCommanded = qHome;
teleopState = hairsim.app.teleop.syncToRobot(teleopState, robotAppState);
ticks = ticks([]);
press(["rightbracket", "leftbracket"]);
assert(all([ticks.status] == "REJECTED:NO_RAIL") && isequal(robotAppState.state.qCurrent, qHome), ...
    "[ ] did something on the fixed base");
fprintf('PASS: Teleop rail slide (%s)\n', options.AdapterName);
result = struct('slides', numel(slides), 'finalThetaDeg', rad2deg(robotAppState.state.railThetaRad));

    function press(keys)
        for key = keys
            teleopState = hairsim.app.teleop.handleKey(teleopState, key, true);
            teleopState = hairsim.app.teleop.handleKey(teleopState, key, false);
            for guard = 1:50
                thetaBefore = robotAppState.state.railThetaRad;
                tcpBefore = tcp();
                [teleopState, poseCommand] = hairsim.app.teleop.stepTeleop(teleopState, robotAppState, true);
                if isempty(poseCommand)
                    break;
                end
                [robotAppState, hairAppState] = hairsim.app.applyPoseTick(poseCommand, robotAppState, ...
                    hairAppState, struct(), [], scene, floorZ, 'Record', false);
                mc = robotAppState.modelContext;
                T = getTransform(mc.robot, robotAppState.state.qCurrent, char(mc.endEffector)) * tool.T_ee_tcp;
                c = hairsim.tool.evaluateClipperClearance(T, tool, teleopState.headQuery, 'CapM', 0.2);
                reported = poseCommand.toolTipWorldM;
                ticks(end + 1) = struct('key', key, 'solved', poseCommand.solved, ...
                    'status', string(poseCommand.statusLabel), 'thetaBefore', thetaBefore, ...
                    'thetaAfter', robotAppState.state.railThetaRad, 'tcpBefore', tcpBefore, ...
                    'tcpAfter', T(1:3, 4)', 'reportedTcp', tipToTcp(reported, T), 'clearanceM', c.minM); %#ok<AGROW>
            end
        end
    end

    function p = tcp()
        mc = robotAppState.modelContext;
        T = getTransform(mc.robot, robotAppState.state.qCurrent, char(mc.endEffector)) * tool.T_ee_tcp;
        p = T(1:3, 4)';
    end

    function p = tipToTcp(reportedTip, T_tcp)
        % the reported mounted tip, mapped back to the TCP with the same
        % rigid offset as the new model's FK (equal iff the model agrees)
        mc = robotAppState.modelContext;
        T_ee = T_tcp / tool.T_ee_tcp;
        offset = T_tcp(1:3, 4)' - hairsim.getMountedClipperPoint(T_ee, mc.toolMount.T_ee_tool);
        p = reportedTip + offset;
    end
end
