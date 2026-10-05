function [qCandidate, T_world_ee, solved, toolTipErrorM, candidateLog, escalationInfo] = ...
        solveRobotForClipperTipWithOrientationFallback(robot, ik, endEffector, weights, ...
        qCurrent, toolTipWorldM, headCenterWorldM, T_ee_tool, maxToolTipErrorM, ...
        headCollisionForIk, toolMountForCollision, collisionMarginMForIk, options)
%SOLVEROBOTFORCLIPPERTIPWITHORIENTATIONFALLBACK Collision-aware IK with tilt escalation.
%   Tier 1 (exact requested orientation: options.BaseRotationWorldEe, or the
%   default head-facing orientation when omitted) is itself staged into
%   increasingly diverse seed sets. ALL Tier 1 stages (1A-1D) are always run
%   to completion, and the globally best-scoring solved-and-collision-free
%   candidate across every stage is returned -- NOT the first stage to reach
%   maxToolTipErrorM. That threshold is a feasibility bound (is this
%   candidate usable at all), not a search-termination criterion: an
%   earlier version of this staging stopped as soon as stage 1A's bare
%   qCurrent seed produced anything under maxToolTipErrorM, which let a
%   ~23mm-error candidate win over a ~7.5mm candidate sitting in stage 1C's
%   seed pool simply because 1A ran first (isolated, single-call
%   before/after regression on the same qCurrent/target/orientation: 1A-only
%   gave 0.023105m via CURRENT_SEED_SUCCESS, running all stages gives
%   0.007509m via DETERMINISTIC_MULTI_SEED_SUCCESS). See the git history
%   around this comment for the diagnosis that caught it.
%     1A CURRENT_SEED: qCurrent alone. The fast path every ordinary stroke
%        should hit.
%     1B LOCAL_PERTURBATION: qCurrent plus a small, FIXED (not
%        random-in-joint-space) bank of qCurrent +/- options.
%        LocalPerturbationEpsilonRad on each of options.RedundantJointIndices.
%        Found by direct diagnosis: a specific qCurrent reached via a real
%        session's own deterministic setup path failed this solve entirely
%        -- Tier 1 AND Tier 2 -- while 20/20 independent
%        perturbations of that exact qCurrent by ~1e-3 rad each solved it
%        instantly via THIS stage alone (median 0.33s). That gap is why this
%        stage exists as its own thing, ahead of 1C: whatever makes this
%        one exact numeric qCurrent hard for the solver (evidence points to
%        a numerically degenerate/solver-sensitive seed -- this has NOT
%        been confirmed as a true robot kinematic singularity, which would
%        need a Jacobian condition-number check this diagnosis did not do)
%        needing only a sub-milliradian nudge to escape is a fundamentally
%        different, far cheaper fix than either the larger structured
%        perturbations in 1C or the ~60-80s tilt search in Tier 2. This
%        perturbs only the IK SEED, never the caller's actual qCurrent --
%        the physical robot is still planned to move from the real qCurrent
%        to whatever solution this stage finds, exactly like every other
%        seed already did.
%     1C DETERMINISTIC_MULTI_SEED: qCurrent plus the all-zero home
%        configuration, options.InitialSeedConfiguration (when
%        options.IncludeInitialSeed), and the deterministic +/-90 degree
%        redundant-joint perturbations of qCurrent on
%        options.RedundantJointIndices (when options.IncludeRedundantSeeds).
%        The home seed is included here UNCONDITIONALLY, regardless of
%        options.IncludeHomeSeed (every current caller passes that false).
%        This matters beyond the one stroke a hard seed hits: a failed
%        stroke's qCurrent never advances, so every later stroke in the
%        same batch repeats the identical failing search from the identical
%        seed -- a single unlucky qCurrent can cascade into 100% of a batch
%        failing, which is a different problem from "these targets are
%        unreachable" and was being misdiagnosed as the latter before this
%        staging existed.
%     1D RANDOM_SEED: stage 1C's seeds plus options.RandomSeedCount
%        additional fixed random joint-space seeds. This was already Tier
%        1's default richness before this staging was added (RandomSeedCount
%        defaults to 12).
%   All four stages run every call; the candidate with the lowest score
%   (toolTipErrorM-dominated, see solveRobotForClipperTip's own scoring)
%   across all of them wins, regardless of which stage produced it.
%   Tier 2 (unchanged from before this staging was added): only entered if
%   NO Tier 1 stage produced any solved-and-collision-free candidate at
%   all. Progressively tilts the tool's local -Z axis away from the exact
%   requested direction (options.TiltAnglesDeg, several tilt directions each
%   via hairsim.createTiltedRotation) and retries, stopping at the smallest
%   tilt level that yields a solved-and-collision-free candidate.
%   Tier 3 (repositioning the robot base) is intentionally NOT implemented
%   here -- escalationInfo.needsBaseReposition signals when even the
%   largest tilt angle fails in every direction, for a human/caller to
%   decide on next steps rather than silently guessing.
%   escalationInfo.searchOutcome records which stage actually produced the
%   returned (globally best) result: "CURRENT_SEED_SUCCESS" /
%   "LOCAL_PERTURBATION_SUCCESS" / "DETERMINISTIC_MULTI_SEED_SUCCESS" /
%   "RANDOM_SEED_SUCCESS" / "TILT_SUCCESS" / "EXHAUSTED". Intended for
%   logging/diagnosis -- callers that only care about
%   solved/needsBaseReposition can ignore it.

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
    headCollisionForIk
    toolMountForCollision
    collisionMarginMForIk (1, 1) double {mustBeNonnegative}
    options.RollAnglesRad (1, :) double {mustBeFinite} = deg2rad([0, -90, 90, 180])
    options.BaseRotationWorldEe double = zeros(0, 0)
    options.IncludeHomeSeed (1, 1) logical = true
    options.IncludeInitialSeed (1, 1) logical = true
    options.IncludeRedundantSeeds (1, 1) logical = true
    options.RandomSeedCount (1, 1) double {mustBeInteger, mustBeNonnegative} = 12
    options.RandomSeedValue (1, 1) double = 12345
    options.LocalPerturbationEpsilonRad (1, 1) double {mustBePositive} = 1e-3
    options.LocalPerturbationSeedCount (1, 1) double {mustBeInteger, mustBeNonnegative} = 12
    options.TiltAnglesDeg (1, :) double {mustBeNonnegative} = [10, 20, 30, 45]
    options.TiltDirectionCount (1, 1) double {mustBeInteger, mustBePositive} = 4
    options.TiltRandomSeedCount (1, 1) double {mustBeInteger, mustBeNonnegative} = 6
    options.RedundantJointIndices (1, :) double {mustBeInteger, mustBePositive} = [1, 3, 5]
    options.InitialSeedConfiguration (:, 1) double = hairsim.motion.createInitialClipperJointConfiguration()
end

if isequal(size(options.BaseRotationWorldEe), [3, 3])
    exactRotationWorldEe = options.BaseRotationWorldEe;
else
    exactRotationWorldEe = hairsim.createHeadFacingRotation(toolTipWorldM, headCenterWorldM);
end

sharedArgs = {robot, ik, endEffector, weights, qCurrent, toolTipWorldM, ...
    headCenterWorldM, T_ee_tool, maxToolTipErrorM};

escalationInfo = struct( ...
    'usedTiltAngleRad', 0, ...
    'usedTiltDirectionRad', 0, ...
    'needsBaseReposition', false, ...
    'searchOutcome', "EXHAUSTED");

localPerturbationSeeds = createLocalPerturbationSeeds( ...
    qCurrent, options.RedundantJointIndices, options.LocalPerturbationEpsilonRad, ...
    options.LocalPerturbationSeedCount, options.RandomSeedValue);

tier1Stages = struct( ...
    'label', {"CURRENT_SEED_SUCCESS", "LOCAL_PERTURBATION_SUCCESS", ...
        "DETERMINISTIC_MULTI_SEED_SUCCESS", "RANDOM_SEED_SUCCESS"}, ...
    'includeHomeSeed', {false, false, true, true}, ...
    'includeInitialSeed', {false, false, options.IncludeInitialSeed, options.IncludeInitialSeed}, ...
    'includeRedundantSeeds', {false, false, options.IncludeRedundantSeeds, options.IncludeRedundantSeeds}, ...
    'randomSeedCount', {0, 0, 0, options.RandomSeedCount}, ...
    'extraSeeds', {zeros(numel(qCurrent), 0), localPerturbationSeeds, ...
        zeros(numel(qCurrent), 0), zeros(numel(qCurrent), 0)});

% Feasibility (maxToolTipErrorM) and search termination are deliberately
% NOT the same test here: every Tier 1 stage runs to completion regardless
% of whether an earlier stage already produced something under
% maxToolTipErrorM, and the globally best-scoring valid candidate across
% all stages is what gets returned.
bestTier1Solved = false;
bestTier1Score = inf;
bestTier1QCandidate = [];
bestTier1TWorldEe = [];
bestTier1ToolTipErrorM = inf;
bestTier1CandidateLog = [];
bestTier1Label = "EXHAUSTED";

for stageIdx = 1:numel(tier1Stages)
    stage = tier1Stages(stageIdx);
    [qCandidateStage, TWorldEeStage, solvedStage, toolTipErrorMStage, candidateLogStage] = ...
        hairsim.motion.solveRobotForClipperTip(sharedArgs{:}, ...
            'RollAnglesRad', options.RollAnglesRad, ...
            'BaseRotationWorldEe', exactRotationWorldEe, ...
            'SeedConfigurations', stage.extraSeeds, ...
            'IncludeHomeSeed', stage.includeHomeSeed, ...
            'IncludeInitialSeed', stage.includeInitialSeed, ...
            'IncludeRedundantSeeds', stage.includeRedundantSeeds, ...
            'RandomSeedCount', stage.randomSeedCount, ...
            'RandomSeedValue', options.RandomSeedValue, ...
            'HeadCollisionForIk', headCollisionForIk, ...
            'ToolMountForCollision', toolMountForCollision, ...
            'CollisionMarginMForIk', collisionMarginMForIk, ...
            'RedundantJointIndices', options.RedundantJointIndices, ...
            'InitialSeedConfiguration', options.InitialSeedConfiguration);
    if solvedStage && candidateLogStage(1).collisionClear ...
            && candidateLogStage(1).score < bestTier1Score
        bestTier1Solved = true;
        bestTier1Score = candidateLogStage(1).score;
        bestTier1QCandidate = qCandidateStage;
        bestTier1TWorldEe = TWorldEeStage;
        bestTier1ToolTipErrorM = toolTipErrorMStage;
        bestTier1CandidateLog = candidateLogStage;
        bestTier1Label = stage.label;
    end
end

% Fallback assignment so the function's declared outputs are always bound
% even when no Tier 1 stage solved anything (matches the raw, unsolved
% last-stage-attempted values a caller would have seen before this
% best-of-all-stages selection existed); overwritten below when a stage did
% produce a valid candidate.
qCandidate = qCandidateStage;
T_world_ee = TWorldEeStage;
solved = solvedStage;
toolTipErrorM = toolTipErrorMStage;
candidateLog = candidateLogStage;

if bestTier1Solved
    qCandidate = bestTier1QCandidate;
    T_world_ee = bestTier1TWorldEe;
    solved = true;
    toolTipErrorM = bestTier1ToolTipErrorM;
    candidateLog = bestTier1CandidateLog;
    escalationInfo.searchOutcome = bestTier1Label;
    return;
end

tiltDirectionsRad = linspace(0, 2 * pi, options.TiltDirectionCount + 1);
tiltDirectionsRad(end) = [];

for tiltAngleDeg = options.TiltAnglesDeg
    tiltAngleRad = deg2rad(tiltAngleDeg);
    bestScore = inf;
    bestQ = [];
    bestT = [];
    bestCandidate = [];
    bestLog = [];
    bestDirectionRad = 0;

    for tiltDirectionRad = tiltDirectionsRad
        tiltedRotation = hairsim.createTiltedRotation( ...
            exactRotationWorldEe, tiltAngleRad, tiltDirectionRad);
        [qAttempt, T_attempt, solvedAttempt, ~, logAttempt] = ...
            hairsim.motion.solveRobotForClipperTip(sharedArgs{:}, ...
                'RollAnglesRad', options.RollAnglesRad, ...
                'BaseRotationWorldEe', tiltedRotation, ...
                'IncludeHomeSeed', options.IncludeHomeSeed, ...
                'IncludeInitialSeed', options.IncludeInitialSeed, ...
                'IncludeRedundantSeeds', options.IncludeRedundantSeeds, ...
                'RandomSeedCount', options.TiltRandomSeedCount, ...
                'RandomSeedValue', options.RandomSeedValue, ...
                'HeadCollisionForIk', headCollisionForIk, ...
                'ToolMountForCollision', toolMountForCollision, ...
                'CollisionMarginMForIk', collisionMarginMForIk, ...
                'RedundantJointIndices', options.RedundantJointIndices, ...
                'InitialSeedConfiguration', options.InitialSeedConfiguration);
        if solvedAttempt && logAttempt(1).collisionClear && logAttempt(1).score < bestScore
            bestScore = logAttempt(1).score;
            bestQ = qAttempt;
            bestT = T_attempt;
            bestCandidate = logAttempt(1);
            bestLog = logAttempt;
            bestDirectionRad = tiltDirectionRad;
        end
    end

    if ~isempty(bestCandidate)
        qCandidate = bestQ;
        T_world_ee = bestT;
        solved = true;
        toolTipErrorM = bestCandidate.toolTipErrorM;
        candidateLog = bestLog;
        escalationInfo.usedTiltAngleRad = tiltAngleRad;
        escalationInfo.usedTiltDirectionRad = bestDirectionRad;
        escalationInfo.searchOutcome = "TILT_SUCCESS";
        return;
    end
end

solved = false;
escalationInfo.needsBaseReposition = true;
escalationInfo.searchOutcome = "EXHAUSTED";
end

function seeds = createLocalPerturbationSeeds( ...
        qCurrent, jointIndices, epsilonRad, combinedSeedCount, randomSeedValue)
%CREATELOCALPERTURBATIONSEEDS qCurrent +/- epsilonRad seed bank on
%   jointIndices: both single-axis (this file's own redundant-joint seeds,
%   just a sub-milliradian-scale step instead of +/-90 degrees) AND
%   combinedSeedCount COMBINED-direction perturbations (every joint in
%   jointIndices nudged at once, independent small offset per joint).
%   The combined perturbations are drawn from a dedicated, FIXED-seed
%   RandStream (randomSeedValue) -- deterministic and reproducible run to
%   run (same inputs always produce the same seed bank), never touching
%   MATLAB's global rng state -- not because "random" is the point, but
%   because it's what was actually shown to work: a single-axis nudge
%   (every other perturbed joint held EXACTLY at qCurrent) and a first
%   attempt at a handful of hand-picked fixed sign patterns (all+, all-,
%   alternating) BOTH empirically failed to escape a solver-sensitive exact
%   seed found during this session's diagnosis, while genuinely independent
%   small per-joint offsets across all of jointIndices at once escaped it
%   20/20 in that same diagnosis. With only a few fixed sign patterns to
%   choose from (4 out of 2^6 possible combinations for a 6-joint arm),
%   there's no principled reason to expect any particular hand-picked one
%   to have a nonzero component along whatever direction actually escapes
%   the degenerate neighborhood -- a small deterministic-seeded RANDOM bank
%   is simply a cheap, reproducible way to get enough directional diversity
%   without having to characterize that neighborhood's exact geometry.
seeds = zeros(numel(qCurrent), 0);
validJoints = intersect(jointIndices, 1:numel(qCurrent));
if isempty(validJoints)
    return;
end

for jointIdx = validJoints
    for signedEpsilon = [epsilonRad, -epsilonRad]
        seed = qCurrent(:);
        seed(jointIdx) = seed(jointIdx) + signedEpsilon;
        seeds(:, end + 1) = seed; %#ok<AGROW>
    end
end

stream = RandStream('mt19937ar', 'Seed', randomSeedValue);
for sampleIdx = 1:combinedSeedCount
    seed = qCurrent(:);
    perturbation = epsilonRad * (2 * rand(stream, numel(validJoints), 1) - 1);
    seed(validJoints) = seed(validJoints) + perturbation;
    seeds(:, end + 1) = seed; %#ok<AGROW>
end
end
