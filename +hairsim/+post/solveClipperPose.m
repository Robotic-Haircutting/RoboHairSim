function [qAccepted, T_actual_tcp, rejected] = solveClipperPose(sctx, params, qPrev, T_world_tcp, tool, headQuery, tier, enforceStep)
%SOLVECLIPPERPOSE One IK sample of the camClipper post-processor: the
%   clipper-model counterpart of hairsim.planning.solveAndPick (which is
%   left untouched for the older planners). The IK target is the clipper
%   TCP pose itself (tool.T_ee_tcp as the tool offset); candidates come
%   from hairsim.motion.solveRobotForClipperTip with the same seeding
%   tiers ("first" adds Home seeds, "extended" adds +-90 deg seeds).
%
%   Gates, in order (first passing candidate wins):
%     solved          IK converged within params.maxToolTipErrorM
%     robotHead       robot links vs head hull + environment (unchanged rule)
%     jointStep       |q - qPrev|_inf <= params.maxJointStepRad (if enforceStep)
%     clipperClearance  clipper contact slab + body vs the EXACT head mesh at
%                     the ACHIEVED pose >= tool.safetyClearanceM
%     selfCollision   arm + clipper body (sctx.armWithToolRobot)
%   T_actual_tcp is the FK pose of the accepted q (what the evaluation and
%   the comb-lift cut use), [] if nothing was accepted.

useHomeSeeds = tier == "first";
useRedundantSeeds = tier ~= "fast";
qAccepted = [];
T_actual_tcp = [];
rejected = struct('notSolved', 0, 'robotHead', 0, 'jointStep', 0, ...
    'clipperClearance', 0, 'selfCollision', 0);

limitsArg = {};
if isfield(sctx, 'revoluteJointLimits')
    limitsArg = {'RevoluteJointLimits', sctx.revoluteJointLimits};
end
[~, ~, ~, ~, candidateLog] = hairsim.motion.solveRobotForClipperTip( ...
    sctx.robot, sctx.ik, sctx.endEffector, params.ikWeights, qPrev, ...
    T_world_tcp(1:3, 4)', sctx.headCenterWorldM, tool.T_ee_tcp, ...
    params.maxToolTipErrorM, ...
    'RollAnglesRad', 0, ...
    'BaseRotationWorldEe', T_world_tcp(1:3, 1:3), ...
    'IncludeHomeSeed', useHomeSeeds, ...
    'IncludeInitialSeed', useHomeSeeds, ...
    'IncludeRedundantSeeds', useRedundantSeeds, ...
    'RandomSeedCount', 0, ...
    'RedundantJointIndices', sctx.redundantJointIndices, ...
    'InitialSeedConfiguration', sctx.qHome, ...
    'HeadCollisionForIk', sctx.headCollision, ...
    'CollisionMarginMForIk', params.collisionMarginM, limitsArg{:});

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
    T_tcp = candidate.TActualWorldEe * tool.T_ee_tcp;
    clearance = hairsim.tool.evaluateClipperClearance(T_tcp, tool, headQuery);
    if ~clearance.ok
        rejected.clipperClearance = rejected.clipperClearance + 1;
        continue;
    end
    if checkCollision(sctx.armWithToolRobot, candidate.q(:), 'SkippedSelfCollisions', 'parent')
        rejected.selfCollision = rejected.selfCollision + 1;
        continue;
    end
    qAccepted = candidate.q(:);
    T_actual_tcp = T_tcp;
    return;
end
end
