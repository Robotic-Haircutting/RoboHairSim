function [qAccepted, rejected] = solveAndPick(ctx, params, ik, qPrev, primitive, k, tier, enforceStep, candidateGenFn)
%SOLVEANDPICK One IK attempt for sample k: tries every (baseRotation,
%   rollAnglesRad) candidate candidateGenFn(R0, params) returns, in order,
%   smallest-deviation-first by the generator's own ordering; returns the
%   first candidate (across every tried orientation) that passes every
%   gate (solved, robot/clipper clear of the head, joint step, clipper-mesh
%   clearance, arm self-collision), or [] plus rejection counts.
%
%   This is the shared per-sample solve both built-in lane-recovery
%   strategies (failLane, bridgeLift) use; candidateGenFn is supplied by
%   whichever orientation-strategy tier is currently active (see
%   hairsim.planning.orientationStrategies.strict for the contract), so
%   orientation and lane-recovery compose independently.

useHomeSeeds = tier == "first";
useRedundantSeeds = tier ~= "fast";
R0 = primitive.R_world_ee(:, :, k);
candidates = candidateGenFn(R0, params);

qAccepted = [];
rejected = struct('notSolved', 0, 'robotHead', 0, 'jointStep', 0, ...
    'clipperClearance', 0, 'selfCollision', 0);
for candSetIdx = 1:numel(candidates)
    [~, ~, ~, ~, candidateLog] = hairsim.motion.solveRobotForClipperTip( ...
        ctx.robot, ik, ctx.endEffector, params.ikWeights, qPrev, ...
        primitive.pointsWorldM(k, :), ctx.headCenterWorldM, ctx.toolMount.T_ee_tool, ...
        params.maxToolTipErrorM, ...
        'RollAnglesRad', candidates(candSetIdx).rollAnglesRad, ...
        'BaseRotationWorldEe', candidates(candSetIdx).baseRotation, ...
        'IncludeHomeSeed', useHomeSeeds, ...
        'IncludeInitialSeed', useHomeSeeds, ...
        'IncludeRedundantSeeds', useRedundantSeeds, ...
        'RandomSeedCount', 0, ...
        'RedundantJointIndices', ctx.redundantJointIndices, ...
        'InitialSeedConfiguration', ctx.qHome, ...
        'HeadCollisionForIk', ctx.headCollision, ...
        'ToolMountForCollision', ctx.toolMount, ...
        'CollisionMarginMForIk', params.collisionMarginM, ...
        'SkipClipperHeadCheckForIk', false);

    for candidateIdx = 1:numel(candidateLog)
        candidate = candidateLog(candidateIdx);
        if ~candidate.solved
            rejected.notSolved = rejected.notSolved + 1;
            continue;
        end
        if ~candidate.collisionClear
            rejected.robotHead = rejected.robotHead + 1;
            continue;
        end
        if enforceStep && max(abs(candidate.q(:) - qPrev(:))) > params.maxJointStepRad
            rejected.jointStep = rejected.jointStep + 1;
            continue;
        end
        centerWorldM = hairsim.getMountedClipperPoint( ...
            candidate.TActualWorldEe, ctx.toolMount.T_ee_tool_center);
        clearance = hairsim.evaluateClipperSphereMeshClearance( ...
            centerWorldM, ctx.headMeshSafetyWorld, ctx.toolMount.toolClearanceRadiusM);
        if clearance.surfaceClearanceM < params.dSafeM
            rejected.clipperClearance = rejected.clipperClearance + 1;
            continue;
        end
        if checkCollision(ctx.armOnlyRobot, candidate.q(:), 'SkippedSelfCollisions', 'parent')
            rejected.selfCollision = rejected.selfCollision + 1;
            continue;
        end
        qAccepted = candidate.q(:);
        return;
    end
end
end
