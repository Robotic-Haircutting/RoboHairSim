function styleProfile = createHairStyleProfile(roots, options)
%CREATEHAIRSTYLEPROFILE Create a procedural middle-part style profile.

arguments
    roots
    options.HairLengthFactor (1, 1) double {mustBePositive} = 1.0
end

rootPositions = extractRootPositions(roots);
if isempty(rootPositions)
    headCenter = [0, 0, 0];
    headHalfSize = [1, 1, 1];
else
    bboxMin = min(rootPositions, [], 1);
    bboxMax = max(rootPositions, [], 1);
    headCenter = (bboxMin + bboxMax) / 2;
    headHalfSize = max((bboxMax - bboxMin) / 2, 1e-6);
end

styleProfile = struct();
styleProfile.name = "hairsim_middle_part";
styleProfile.headCenter = headCenter;
styleProfile.headHalfSize = headHalfSize;
styleProfile.baseHairLength = 0.42 * max(headHalfSize);
styleProfile.hairLengthFactor = options.HairLengthFactor;
styleProfile.rootClearance = 0.0025;
styleProfile.partGapWidth = 0.018;
styleProfile.partTransitionWidth = 0.08;
styleProfile.flowCurveScale = 0.42;
styleProfile.gravityCurveScale = 1.15;
styleProfile.gravityStartFraction = 0.18;
styleProfile.topLiftScale = 0.22;

styleProfile.nodeProfile = struct();
styleProfile.nodeProfile.rootStiffnessScale = 1.0;
styleProfile.nodeProfile.guideExponent = 1.20;
styleProfile.nodeProfile.gravityExponent = 1.35;
styleProfile.nodeProfile.bendSoftnessRoot = 0.25;
styleProfile.nodeProfile.bendSoftnessTip = 0.95;

styleProfile.regionProfiles = struct();
styleProfile.regionProfiles.scalp_top_left = makeRegion([0.48, 0.70], [0.70, 0.15, -0.18], 0.55, 0.82, 0.30);
styleProfile.regionProfiles.scalp_top_right = makeRegion([0.48, 0.70], [-0.70, 0.15, -0.18], 0.55, 0.82, 0.30);
styleProfile.regionProfiles.scalp_left = makeRegion([0.38, 0.58], [0.28, 0.02, -0.96], 1.15, 0.45, 0.55);
styleProfile.regionProfiles.scalp_right = makeRegion([0.38, 0.58], [-0.28, 0.02, -0.96], 1.15, 0.45, 0.55);
styleProfile.regionProfiles.scalp_back = makeRegion([0.48, 0.74], [0.00, -0.28, -0.96], 1.25, 0.42, 0.65);
end

function rootPositions = extractRootPositions(roots)
if isempty(roots)
    rootPositions = zeros(0, 3);
    return;
end

rootPositions = zeros(numel(roots), 3);
for rootIdx = 1:numel(roots)
    rootPositions(rootIdx, :) = double(roots(rootIdx).position(:)');
end
end

function regionProfile = makeRegion(lengthRange, flow, gravityScale, guideAttractionScale, bendSoftnessScale)
regionProfile = struct();
regionProfile.lengthRange = lengthRange;
regionProfile.flow = hairsim.normalizeVector(double(flow(:)'), [0, 0, -1]);
regionProfile.gravityScale = gravityScale;
regionProfile.guideAttractionScale = guideAttractionScale;
regionProfile.bendSoftnessScale = bendSoftnessScale;
end
