function [qCandidate, T_world_ee, solved, toolTipErrorM, candidateLog] = ...
        solveRobotForClipperTip(robot, ik, endEffector, weights, qCurrent, ...
        toolTipWorldM, headCenterWorldM, T_ee_tool, maxToolTipErrorM, options)
%SOLVEROBOTFORCLIPPERTIP Solve relaxed multi-seed IK for a clipper tip.
%   options.HeadCollisionForIk: collision objects the robot LINKS (not the
%   clipper) must clear; the clipper's own clearance is the caller's gate
%   (hairsim.tool.evaluateClipperClearance).

arguments
    robot
    ik
    endEffector (1, 1) string
    weights (1, :) double {mustBeFinite}
    qCurrent (:, 1) double {mustBeFinite}
    toolTipWorldM (1, 3) double {mustBeFinite}
    headCenterWorldM (1, 3) double {mustBeFinite}
    T_ee_tool (4, 4) double {mustBeFinite}
    maxToolTipErrorM (1, 1) double {mustBeNonnegative}
    options.RollAnglesRad (1, :) double {mustBeFinite} = deg2rad([0, -90, 90, 180])
    options.SeedConfigurations double = zeros(0, 0)
    options.IncludeHomeSeed (1, 1) logical = true
    options.IncludeInitialSeed (1, 1) logical = true
    options.IncludeRedundantSeeds (1, 1) logical = true
    options.JointStepWeight (1, 1) double {mustBeNonnegative} = 0.02
    options.OrientationWeight (1, 1) double {mustBeNonnegative} = 0.001
    options.ApproachAxisWeight (1, 1) double {mustBeNonnegative} = 1.0
    options.ForwardAxisWeight (1, 1) double {mustBeNonnegative} = 1.0
    options.BaseRotationWorldEe double = zeros(0, 0)
    options.HeadCollisionForIk = []
    options.CollisionMarginMForIk (1, 1) double {mustBeNonnegative} = 0.001
    options.RandomSeedCount (1, 1) double {mustBeInteger, mustBeNonnegative} = 0
    options.RandomSeedValue (1, 1) double = 12345
    options.RedundantJointIndices (1, :) double {mustBeInteger, mustBePositive} = [1, 3, 5, 7]
    options.InitialSeedConfiguration (:, 1) double = hairsim.motion.createInitialClipperJointConfiguration()
end

if isequal(size(options.BaseRotationWorldEe), [3, 3])
    baseRotationWorldEe = options.BaseRotationWorldEe;
else
    baseRotationWorldEe = hairsim.createHeadFacingRotation(toolTipWorldM, headCenterWorldM);
end
seedConfigurations = createIkSeedConfigurations( ...
    robot, qCurrent, options.SeedConfigurations, ...
    options.IncludeHomeSeed, options.IncludeInitialSeed, ...
    options.IncludeRedundantSeeds, options.RandomSeedCount, options.RandomSeedValue, ...
    options.RedundantJointIndices, options.InitialSeedConfiguration);
candidateLog = createEmptyCandidateLog(numel(qCurrent));

for rollIdx = 1:numel(options.RollAnglesRad)
    rollAngleRad = options.RollAnglesRad(rollIdx);
    R_world_ee = baseRotationWorldEe * rotationAboutLocalZ(rollAngleRad);
    T_target = hairsim.motion.createEndEffectorTransformForToolTip( ...
        toolTipWorldM, R_world_ee, T_ee_tool);
    for seedIdx = 1:size(seedConfigurations, 2)
        candidateLog(end + 1) = solveOneCandidate( ...
            robot, ik, endEffector, weights, seedConfigurations(:, seedIdx), ...
            qCurrent, T_target, T_ee_tool, toolTipWorldM, rollAngleRad, ...
            seedIdx, maxToolTipErrorM, options.JointStepWeight, ...
            options.OrientationWeight, options.ApproachAxisWeight, ...
            options.ForwardAxisWeight, baseRotationWorldEe, options.HeadCollisionForIk, ...
            options.CollisionMarginMForIk); %#ok<AGROW>
    end
end

candidateLog = sortCandidateLog(candidateLog);
if isempty(candidateLog)
    qCandidate = nan(size(qCurrent));
    T_world_ee = hairsim.motion.createEndEffectorTransformForToolTip( ...
        toolTipWorldM, baseRotationWorldEe, T_ee_tool);
    toolTipErrorM = inf;
    solved = false;
    return;
end

qCandidate = candidateLog(1).q;
T_world_ee = candidateLog(1).TTargetWorldEe;
toolTipErrorM = candidateLog(1).toolTipErrorM;
solved = candidateLog(1).solved;
end

function candidate = solveOneCandidate(robot, ik, endEffector, weights, seed, ...
        qCurrent, T_target, T_ee_tool, toolTipWorldM, rollAngleRad, seedIdx, ...
        maxToolTipErrorM, jointStepWeight, orientationWeight, approachAxisWeight, ...
        forwardAxisWeight, baseRotationWorldEe, headCollisionForIk, collisionMarginMForIk)
candidate = createCandidate(numel(qCurrent));
candidate.TTargetWorldEe = T_target;
candidate.rollAngleRad = rollAngleRad;
candidate.seedIndex = seedIdx;

try
    qSolution = ik(char(endEffector), T_target, weights, seed(:));
catch
    qSolution = nan(size(qCurrent));
end
qSolution = qSolution(:);
qSolution = normalizeJointConfigurationNearReference(robot, qSolution, qCurrent(:));
candidate.q = qSolution;

if ~all(isfinite(qSolution))
    return;
end

T_actual = getTransform(robot, qSolution, char(endEffector));
actualToolTipWorldM = hairsim.getMountedClipperPoint(T_actual, T_ee_tool);
candidate.TActualWorldEe = T_actual;
candidate.actualToolTipWorldM = actualToolTipWorldM;
candidate.toolTipErrorM = norm(actualToolTipWorldM - toolTipWorldM);
candidate.orientationErrorRad = rotationErrorRad( ...
    T_actual(1:3, 1:3), T_target(1:3, 1:3));
% Task-axis errors are measured against baseRotationWorldEe (the canonical,
% NOT-yet-roll-shifted desired orientation: a_T=baseRotationWorldEe(:,3),
% f_T=baseRotationWorldEe(:,1)), NOT against T_target -- T_target already
% has this candidate's own rollAngleRad baked in
% (T_target = baseRotationWorldEe * rotationAboutLocalZ(rollAngleRad)), so
% comparing against it would make approachAxisErrorRad/forwardAxisErrorRad
% trivially ~0 for ANY well-converged candidate regardless of which roll it
% tried, defeating the entire point of scoring roll selection by task
% semantics. (Diagnosed: this was exactly the bug in this score term's
% first version -- confirmed via a stroke whose top-scored candidate was a
% -90deg roll with both "error" terms reading 0.0.)
[candidate.approachAxisErrorRad, candidate.forwardAxisErrorRad] = ...
    axisAlignedErrorsRad(T_actual(1:3, 1:3), baseRotationWorldEe);
candidate.jointStepRad = norm(qSolution - qCurrent(:));
candidate.solved = candidate.toolTipErrorM <= maxToolTipErrorM;

if candidate.solved && ~isempty(headCollisionForIk)
    [collisionClearFlag, ~, collisionSepDistM] = hairsim.motion.validateClipperRobotCollision( ...
        robot, qSolution, endEffector, headCollisionForIk, collisionMarginMForIk, true);
    candidate.collisionClear = collisionClearFlag;
    candidate.collisionSepDistM = collisionSepDistM;
end

candidate.score = candidate.toolTipErrorM + ...
    jointStepWeight * candidate.jointStepRad + ...
    orientationWeight * candidate.orientationErrorRad + ...
    approachAxisWeight * candidate.approachAxisErrorRad + ...
    forwardAxisWeight * candidate.forwardAxisErrorRad;
if ~candidate.solved
    candidate.score = candidate.score + maxToolTipErrorM;
elseif ~candidate.collisionClear
    candidate.score = candidate.score + 1000;
end
end

function seedConfigurations = createIkSeedConfigurations(robot, qCurrent, extraSeeds, ...
        includeHomeSeed, includeInitialSeed, includeRedundantSeeds, ...
        randomSeedCount, randomSeedValue, redundantJointIndices, initialSeedConfiguration)
numJoints = numel(qCurrent);
seedConfigurations = qCurrent(:);
if includeHomeSeed
    seedConfigurations(:, end + 1) = zeros(numJoints, 1);
end
if includeInitialSeed
    if numel(initialSeedConfiguration) == numJoints
        seedConfigurations(:, end + 1) = initialSeedConfiguration(:);
    end
end
if ~isempty(extraSeeds)
    seedConfigurations = appendExtraSeeds(seedConfigurations, extraSeeds, numJoints);
end

if includeRedundantSeeds
    seedConfigurations = [seedConfigurations, ...
        createRedundantJointSeeds(qCurrent(:), redundantJointIndices)];
end
if randomSeedCount > 0
    seedConfigurations = [seedConfigurations, ...
        createRandomJointSeeds(robot, numJoints, randomSeedCount, randomSeedValue)];
end
seedConfigurations = wrapAndClampSeeds(robot, seedConfigurations);
seedConfigurations = uniqueSeedColumns(seedConfigurations);
end

function seedConfigurations = createRandomJointSeeds(robot, numJoints, count, seedValue)
% Dispersed joint-space seeds sampled from an independent RandStream (does
% not touch the global rng state used elsewhere, e.g. hairstyle generation).
seedConfigurations = zeros(numJoints, count);
stream = RandStream('mt19937ar', 'Seed', seedValue);
jointIdx = 0;
for bodyIdx = 1:robot.NumBodies
    joint = robot.Bodies{bodyIdx}.Joint;
    if ~strcmp(joint.Type, 'revolute')
        continue;
    end
    jointIdx = jointIdx + 1;
    if jointIdx > numJoints
        break;
    end
    limits = joint.PositionLimits;
    if all(isfinite(limits))
        seedConfigurations(jointIdx, :) = limits(1) + ...
            (limits(2) - limits(1)) * rand(stream, 1, count);
    else
        seedConfigurations(jointIdx, :) = -pi + 2 * pi * rand(stream, 1, count);
    end
end
end

function qNear = normalizeJointConfigurationNearReference(robot, qRaw, qReference)
qNear = qRaw(:);
jointIdx = 0;
for bodyIdx = 1:robot.NumBodies
    joint = robot.Bodies{bodyIdx}.Joint;
    if ~strcmp(joint.Type, 'revolute')
        continue;
    end
    jointIdx = jointIdx + 1;
    if jointIdx > numel(qNear)
        break;
    end
    qNear(jointIdx) = chooseEquivalentJointAngleNearReference( ...
        qNear(jointIdx), qReference(jointIdx), joint.PositionLimits);
end
end

function qNear = chooseEquivalentJointAngleNearReference(qRaw, qReference, limits)
equivalentAngles = qRaw + 2 * pi * (-4:4);
if all(isfinite(limits))
    inLimit = equivalentAngles >= limits(1) & equivalentAngles <= limits(2);
    equivalentAngles = equivalentAngles(inLimit);
end
if isempty(equivalentAngles)
    qNear = min(max(wrapToPi(qRaw), limits(1)), limits(2));
    return;
end
[~, bestIdx] = min(abs(equivalentAngles - qReference));
qNear = equivalentAngles(bestIdx);
end

function seedConfigurations = appendExtraSeeds(seedConfigurations, extraSeeds, numJoints)
if size(extraSeeds, 1) == numJoints
    seedConfigurations = [seedConfigurations, extraSeeds];
elseif size(extraSeeds, 2) == numJoints
    seedConfigurations = [seedConfigurations, extraSeeds'];
end
end

function seedConfigurations = createRedundantJointSeeds(qCurrent, redundantJointIndices)
seedConfigurations = zeros(numel(qCurrent), 0);
redundantJoints = intersect(redundantJointIndices, 1:numel(qCurrent));
for jointIdx = redundantJoints
    for deltaRad = [pi / 2, -pi / 2]
        seed = qCurrent;
        seed(jointIdx) = seed(jointIdx) + deltaRad;
        seedConfigurations(:, end + 1) = seed; %#ok<AGROW>
    end
end
end

function seeds = wrapAndClampSeeds(robot, seeds)
seeds = wrapToPi(seeds);
finiteJointIdx = 0;
for bodyIdx = 1:robot.NumBodies
    joint = robot.Bodies{bodyIdx}.Joint;
    if ~strcmp(joint.Type, 'revolute')
        continue;
    end
    finiteJointIdx = finiteJointIdx + 1;
    limits = joint.PositionLimits;
    if all(isfinite(limits))
        seeds(finiteJointIdx, :) = min(max(seeds(finiteJointIdx, :), limits(1)), limits(2));
    end
end
end

function uniqueSeeds = uniqueSeedColumns(seeds)
uniqueSeeds = zeros(size(seeds, 1), 0);
for seedIdx = 1:size(seeds, 2)
    seed = seeds(:, seedIdx);
    if isempty(uniqueSeeds) || all(vecnorm(uniqueSeeds - seed, 2, 1) > 1e-6)
        uniqueSeeds(:, end + 1) = seed; %#ok<AGROW>
    end
end
end

function candidateLog = sortCandidateLog(candidateLog)
if isempty(candidateLog)
    return;
end
scores = [candidateLog.score];
[~, order] = sort(scores);
candidateLog = candidateLog(order);
end

function candidateLog = createEmptyCandidateLog(numJoints)
candidate = createCandidate(numJoints);
candidateLog = candidate([]);
end

function candidate = createCandidate(numJoints)
candidate = struct( ...
    'q', zeros(numJoints, 1), ...
    'TTargetWorldEe', nan(4, 4), ...
    'TActualWorldEe', nan(4, 4), ...
    'actualToolTipWorldM', [nan, nan, nan], ...
    'toolTipErrorM', inf, ...
    'orientationErrorRad', inf, ...
    'approachAxisErrorRad', inf, ...
    'forwardAxisErrorRad', inf, ...
    'jointStepRad', inf, ...
    'rollAngleRad', 0, ...
    'seedIndex', 0, ...
    'solved', false, ...
    'collisionClear', true, ...
    'collisionSepDistM', inf, ...
    'score', inf);
end

function R = rotationAboutLocalZ(angleRad)
R = [
    cos(angleRad), -sin(angleRad), 0
    sin(angleRad),  cos(angleRad), 0
    0,              0,             1
    ];
end

function angleRad = rotationErrorRad(R_actual, R_target)
rotationDelta = R_target' * R_actual;
cosAngle = (trace(rotationDelta) - 1) / 2;
cosAngle = min(1, max(-1, cosAngle));
angleRad = acos(cosAngle);
end

function [approachAxisErrorRad, forwardAxisErrorRad] = axisAlignedErrorsRad(R_actual, R_target)
%AXISALIGNEDERRORSRAD Decompose orientation error into the two task-relevant
%   components, per hairsim.createSurfaceFollowingRotation's own axis
%   convention (3rd column = approach axis a_T, built as -normal; 1st
%   column = forward/cutting axis f_T, built as the tangent):
%     approachAxisErrorRad: angle between R_actual's and R_target's 3rd
%       column (a_T tilt away from -normal).
%     forwardAxisErrorRad: residual twist about R_target's approach axis
%       needed to align R_actual's 1st column with R_target's (f_T
%       deviation from the tangent, after accounting for any approach-axis
%       tilt). This is the quantity a fixed RollAnglesRad candidate set
%       varies almost for free (position/approach-axis alignment barely
%       constrains it), so it needs its own explicit score term -- see the
%       diagnosis this was added for: 23/36 strokes in one benchmark run
%       picked a -90deg roll candidate with forwardAxisErrorRad~=90deg
%       while approachAxisErrorRad stayed exactly 0, because the previous
%       single scalar orientationErrorRad (full SO(3) geodesic angle,
%       weighted at 0.001) barely penalized it relative to jointStepRad.
aActual = R_actual(:, 3);
aTarget = R_target(:, 3);
approachAxisErrorRad = acos(min(1, max(-1, dot(aActual, aTarget))));

fActual = R_actual(:, 1);
fTarget = R_target(:, 1);
fActualProjected = fActual - dot(fActual, aTarget) * aTarget;
fActualProjectedNorm = norm(fActualProjected);
if fActualProjectedNorm < 1e-8
    forwardAxisErrorRad = 0;
    return;
end
fActualProjected = fActualProjected / fActualProjectedNorm;
cosRoll = dot(fActualProjected, fTarget);
sinRoll = dot(cross(fTarget, fActualProjected), aTarget);
forwardAxisErrorRad = abs(atan2(sinRoll, cosRoll));
end
