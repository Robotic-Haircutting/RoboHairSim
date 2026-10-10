function hairState = stepGuideHairPhysics(hairState, options)
%STEPGUIDEHAIRPHYSICS Advance dynamic guide hair by one simulation step.

arguments
    hairState (1, 1) struct
    options.Gravity (1, 3) double = [0, 0, -0.0015]
    options.Rigidity (1, 1) double {mustBeNonnegative} = 0.5
    options.Damping (1, 1) double {mustBeNonnegative} = 0.95
    options.Dt (1, 1) double {mustBePositive} = 0.8
    options.BendStiffness (1, 1) double {mustBeNonnegative} = 0.05
    options.LengthConstraintStiffness (1, 1) double {mustBeNonnegative} = 0.85
    options.StrandIndices (1, :) double {mustBeInteger, mustBePositive} = []
    % bend + length + root pin are repeated this many times per step: one
    % pass corrects only part of the stretch gravity adds each step, so a
    % single pass lets strands creep to about twice their length
    options.ConstraintIterations (1, 1) double {mustBeInteger, mustBePositive} = 1
end

[nodesPerStrand, numStrands] = size(hairState.hairs.X);
strandIndices = hairsim.hair.dynamics.createStrandIndexList(numStrands, options.StrandIndices);
for strandIdx = strandIndices
    activeCount = min(hairState.activeNodeCounts(strandIdx), nodesPerStrand);
    if activeCount < 2
        continue;
    end

    for nodeIdx = 2:activeCount
        guideForce = [ ...
            hairState.guideTargets.X(nodeIdx, strandIdx) - hairState.hairs.X(nodeIdx, strandIdx), ...
            hairState.guideTargets.Y(nodeIdx, strandIdx) - hairState.hairs.Y(nodeIdx, strandIdx), ...
            hairState.guideTargets.Z(nodeIdx, strandIdx) - hairState.hairs.Z(nodeIdx, strandIdx)] .* ...
            (options.Rigidity * 0.35 * hairState.guideAttraction(nodeIdx, strandIdx));
        gravityForce = options.Gravity .* ...
            (0.25 + hairState.gravityWeight(nodeIdx, strandIdx));

        hairState.vel.X(nodeIdx, strandIdx) = hairState.vel.X(nodeIdx, strandIdx) + ...
            guideForce(1) + gravityForce(1);
        hairState.vel.Y(nodeIdx, strandIdx) = hairState.vel.Y(nodeIdx, strandIdx) + ...
            guideForce(2) + gravityForce(2);
        hairState.vel.Z(nodeIdx, strandIdx) = hairState.vel.Z(nodeIdx, strandIdx) + ...
            guideForce(3) + gravityForce(3);

        hairState.hairs.X(nodeIdx, strandIdx) = hairState.hairs.X(nodeIdx, strandIdx) + ...
            hairState.vel.X(nodeIdx, strandIdx) * options.Dt;
        hairState.hairs.Y(nodeIdx, strandIdx) = hairState.hairs.Y(nodeIdx, strandIdx) + ...
            hairState.vel.Y(nodeIdx, strandIdx) * options.Dt;
        hairState.hairs.Z(nodeIdx, strandIdx) = hairState.hairs.Z(nodeIdx, strandIdx) + ...
            hairState.vel.Z(nodeIdx, strandIdx) * options.Dt;

        hairState.vel.X(nodeIdx, strandIdx) = hairState.vel.X(nodeIdx, strandIdx) * options.Damping;
        hairState.vel.Y(nodeIdx, strandIdx) = hairState.vel.Y(nodeIdx, strandIdx) * options.Damping;
        hairState.vel.Z(nodeIdx, strandIdx) = hairState.vel.Z(nodeIdx, strandIdx) * options.Damping;
    end

    for iteration = 1:options.ConstraintIterations
        hairState = applyBendConstraint(hairState, strandIdx, activeCount, options.BendStiffness);
        hairState = applyLengthConstraint(hairState, strandIdx, activeCount, ...
            options.LengthConstraintStiffness);
        hairState = hairsim.hair.dynamics.pinRootToGuide(hairState, strandIdx);
    end
end
end

function hairState = applyBendConstraint(hairState, strandIdx, activeCount, bendStiffness)
if activeCount < 3 || bendStiffness <= 0
    return;
end
for nodeIdx = 2:(activeCount - 1)
    previousNode = getHairNode(hairState.hairs, nodeIdx - 1, strandIdx);
    currentNode = getHairNode(hairState.hairs, nodeIdx, strandIdx);
    nextNode = getHairNode(hairState.hairs, nodeIdx + 1, strandIdx);
    targetMidpoint = (previousNode + nextNode) / 2;
    softness = hairState.bendSoftness(nodeIdx, strandIdx);
    correction = (targetMidpoint - currentNode) * bendStiffness * (1.15 - 0.65 * softness);
    hairState = addHairNodeOffset(hairState, nodeIdx, strandIdx, correction);
end
end

function hairState = applyLengthConstraint(hairState, strandIdx, activeCount, stiffness)
targetLength = hairState.targetSegmentLengths(strandIdx);
if targetLength <= eps || stiffness <= 0
    return;
end
for nodeIdx = 2:activeCount
    previousNode = getHairNode(hairState.hairs, nodeIdx - 1, strandIdx);
    currentNode = getHairNode(hairState.hairs, nodeIdx, strandIdx);
    segmentVector = currentNode - previousNode;
    segmentLength = norm(segmentVector);
    if segmentLength <= eps
        continue;
    end
    correction = (segmentLength - targetLength) / segmentLength * stiffness * segmentVector;
    hairState = addHairNodeOffset(hairState, nodeIdx, strandIdx, -correction);
end
end

function node = getHairNode(hairs, nodeIdx, strandIdx)
node = [hairs.X(nodeIdx, strandIdx), hairs.Y(nodeIdx, strandIdx), hairs.Z(nodeIdx, strandIdx)];
end

function hairState = addHairNodeOffset(hairState, nodeIdx, strandIdx, offset)
hairState.hairs.X(nodeIdx, strandIdx) = hairState.hairs.X(nodeIdx, strandIdx) + offset(1);
hairState.hairs.Y(nodeIdx, strandIdx) = hairState.hairs.Y(nodeIdx, strandIdx) + offset(2);
hairState.hairs.Z(nodeIdx, strandIdx) = hairState.hairs.Z(nodeIdx, strandIdx) + offset(3);
end
