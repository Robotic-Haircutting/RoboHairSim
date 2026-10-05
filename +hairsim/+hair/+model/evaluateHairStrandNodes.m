function strandNodes = evaluateHairStrandNodes(root, normal, region, ...
    styleProfile, strandLength, nodesPerStrand)
%EVALUATEHAIRSTRANDNODES Build a fixed-segment procedural guide strand.

arguments
    root (1, 3) double
    normal (1, 3) double
    region
    styleProfile struct
    strandLength (1, 1) double {mustBeNonnegative}
    nodesPerStrand (1, 1) double {mustBeInteger, mustBeGreaterThan(nodesPerStrand, 1)}
end

surfaceNormal = hairsim.normalizeVector(normal, [0, 0, 1]);
strandNodes = zeros(nodesPerStrand, 3);
strandNodes(1, :) = root + styleProfile.rootClearance * surfaceNormal;

segmentLength = strandLength / (nodesPerStrand - 1);
baseFlow = getRegionFlow(styleProfile, region, root);
baseFlow = projectToTangent(baseFlow, surfaceNormal);
baseFlow = hairsim.normalizeVector(baseFlow, [signNonZero(root(1) - styleProfile.headCenter(1)), 0, -0.2]);
worldGravity = [0, 0, -1];

for nodeIdx = 2:nodesPerStrand
    strandFraction = (nodeIdx - 1) / (nodesPerStrand - 1);
    smoothFraction = strandFraction ^ 2 * (3 - 2 * strandFraction);
    localNormal = estimateHeadEllipsoidNormal(strandNodes(nodeIdx - 1, :), ...
        styleProfile, surfaceNormal);
    flow = hairsim.normalizeVector(projectToTangent(baseFlow, localNormal), baseFlow);
    downhill = projectToTangent(worldGravity, localNormal);
    downhill = hairsim.normalizeVector(downhill, worldGravity);

    rootLiftWeight = styleProfile.topLiftScale * exp(-8.0 * strandFraction);
    flowWeight = styleProfile.flowCurveScale * smoothFraction * ...
        partLineFlowGate(root, styleProfile);
    gravityWeight = styleProfile.gravityCurveScale * ...
        max(0, strandFraction - styleProfile.gravityStartFraction) ^ 1.35;
    direction = hairsim.normalizeVector( ...
        rootLiftWeight * localNormal + ...
        flowWeight * flow + ...
        gravityWeight * downhill + ...
        0.04 * surfaceNormal, downhill);

    strandNodes(nodeIdx, :) = strandNodes(nodeIdx - 1, :) + ...
        segmentLength * direction;
end
end

function flow = getRegionFlow(styleProfile, region, root)
regionName = char(string(region));
if isfield(styleProfile.regionProfiles, regionName)
    flow = styleProfile.regionProfiles.(regionName).flow;
    return;
end

flow = [signNonZero(root(1) - styleProfile.headCenter(1)), 0.10, -0.55];
end

function gate = partLineFlowGate(root, styleProfile)
distance = abs(root(1) - styleProfile.headCenter(1));
gate = min(1, max(0, (distance - styleProfile.partGapWidth) / ...
    max(styleProfile.partTransitionWidth, eps)));
gate = gate * gate * (3 - 2 * gate);
end

function normal = estimateHeadEllipsoidNormal(point, styleProfile, defaultNormal)
scaledPoint = (point - styleProfile.headCenter) ./ max(styleProfile.headHalfSize, eps);
if norm(scaledPoint) < eps
    normal = defaultNormal;
    return;
end
normalApprox = scaledPoint ./ max(styleProfile.headHalfSize, eps);
normal = hairsim.normalizeVector(normalApprox, defaultNormal);
end

function tangent = projectToTangent(vector, normal)
tangent = vector - dot(vector, normal) * normal;
end

function signValue = signNonZero(value)
if value < 0
    signValue = -1;
else
    signValue = 1;
end
end
