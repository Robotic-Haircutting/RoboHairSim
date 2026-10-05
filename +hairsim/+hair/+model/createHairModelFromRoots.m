function hairModel = createHairModelFromRoots(rootsSource, options)
%CREATEHAIRMODELFROMROOTS Build standalone guide-hair geometry from roots.

arguments
    rootsSource
    options.NumStrands (1, 1) double {mustBeInteger, mustBeNonnegative} = 0
    options.NodesPerStrand (1, 1) double {mustBeInteger, mustBeGreaterThan(options.NodesPerStrand, 1)} = 12
    options.HairLengthFactor (1, 1) double {mustBePositive} = 1.0
    options.RandomSeed (1, 1) double = 11
end

roots = loadRootsSource(rootsSource);
if isempty(roots)
    error("hairsim:NoHairRoots", "Hair model requires at least one root.");
end

requestedStrands = options.NumStrands;
if requestedStrands == 0
    requestedStrands = numel(roots);
end
numStrands = min(requestedStrands, numel(roots));
selectedRoots = roots(1:numStrands);

styleProfile = hairsim.hair.model.createHairStyleProfile(selectedRoots, ...
    'HairLengthFactor', options.HairLengthFactor);
rng(options.RandomSeed);

[rootPositions, rootNormals, rootRegions] = extractRootMatrices(selectedRoots);
strandLengthScale = sampleStrandLengthScales(rootRegions, styleProfile);
localHairLengths = styleProfile.baseHairLength * options.HairLengthFactor * strandLengthScale(:)';
targetSegmentLengths = localHairLengths / (options.NodesPerStrand - 1);

guideX = zeros(options.NodesPerStrand, numStrands);
guideY = zeros(options.NodesPerStrand, numStrands);
guideZ = zeros(options.NodesPerStrand, numStrands);
guideAttraction = zeros(options.NodesPerStrand, numStrands);
gravityWeight = zeros(options.NodesPerStrand, numStrands);
bendSoftness = zeros(options.NodesPerStrand, numStrands);

for strandIdx = 1:numStrands
    strandNodes = hairsim.hair.model.evaluateHairStrandNodes( ...
        rootPositions(strandIdx, :), rootNormals(strandIdx, :), ...
        rootRegions(strandIdx), styleProfile, localHairLengths(strandIdx), ...
        options.NodesPerStrand);

    guideX(:, strandIdx) = strandNodes(:, 1);
    guideY(:, strandIdx) = strandNodes(:, 2);
    guideZ(:, strandIdx) = strandNodes(:, 3);

    for nodeIdx = 1:options.NodesPerStrand
        strandFraction = (nodeIdx - 1) / (options.NodesPerStrand - 1);
        nodeProfile = evaluateNodeProfile(styleProfile, rootRegions(strandIdx), strandFraction);
        guideAttraction(nodeIdx, strandIdx) = nodeProfile.guideAttraction;
        gravityWeight(nodeIdx, strandIdx) = nodeProfile.gravityWeight;
        bendSoftness(nodeIdx, strandIdx) = nodeProfile.bendSoftness;
    end
end

hairModel = struct();
hairModel.numStrands = numStrands;
hairModel.nodesPerStrand = options.NodesPerStrand;
hairModel.roots = rootPositions;
hairModel.normals = rootNormals;
hairModel.regions = rootRegions;
hairModel.styleProfile = styleProfile;
hairModel.strandLengthScale = strandLengthScale(:)';
hairModel.localHairLengths = localHairLengths;
hairModel.targetSegmentLengths = targetSegmentLengths;
hairModel.guideX = guideX;
hairModel.guideY = guideY;
hairModel.guideZ = guideZ;
hairModel.guideAttraction = guideAttraction;
hairModel.gravityWeight = gravityWeight;
hairModel.bendSoftness = bendSoftness;
[lineX, lineY, lineZ] = createGuideLineData(hairModel);
hairModel.lineX = lineX;
hairModel.lineY = lineY;
hairModel.lineZ = lineZ;
end

function [lineX, lineY, lineZ] = createGuideLineData(hairModel)
lineLength = hairModel.numStrands * (hairModel.nodesPerStrand + 1);
lineX = nan(lineLength, 1);
lineY = nan(lineLength, 1);
lineZ = nan(lineLength, 1);

cursor = 1;
for strandIdx = 1:hairModel.numStrands
    strandRange = cursor:(cursor + hairModel.nodesPerStrand - 1);
    lineX(strandRange) = hairModel.guideX(:, strandIdx);
    lineY(strandRange) = hairModel.guideY(:, strandIdx);
    lineZ(strandRange) = hairModel.guideZ(:, strandIdx);
    cursor = cursor + hairModel.nodesPerStrand + 1;
end
end

function roots = loadRootsSource(rootsSource)
if ischar(rootsSource) || isstring(rootsSource)
    roots = hairsim.hair.model.loadHairRoots(char(rootsSource));
else
    roots = rootsSource;
end
end

function [rootPositions, rootNormals, rootRegions] = extractRootMatrices(roots)
numStrands = numel(roots);
rootPositions = zeros(numStrands, 3);
rootNormals = zeros(numStrands, 3);
rootRegions = strings(numStrands, 1);

for rootIdx = 1:numStrands
    rootPositions(rootIdx, :) = double(roots(rootIdx).position(:)');
    normal = double(roots(rootIdx).normal(:)');
    rootNormals(rootIdx, :) = hairsim.normalizeVector(normal, [0, 0, 1]);
    rootRegions(rootIdx) = string(roots(rootIdx).region);
end
end

function strandLengthScale = sampleStrandLengthScales(regions, styleProfile)
strandLengthScale = ones(numel(regions), 1);
for strandIdx = 1:numel(regions)
    regionProfile = getRegionProfile(styleProfile, regions(strandIdx));
    lengthRange = regionProfile.lengthRange;
    strandLengthScale(strandIdx) = lengthRange(1) + ...
        (lengthRange(2) - lengthRange(1)) * rand;
end
end

function nodeProfile = evaluateNodeProfile(styleProfile, region, strandFraction)
t = max(0, min(1, strandFraction));
regionProfile = getRegionProfile(styleProfile, region);

nodeProfile = struct();
nodeProfile.guideAttraction = regionProfile.guideAttractionScale * ...
    (1 - t) ^ styleProfile.nodeProfile.guideExponent;
nodeProfile.gravityWeight = regionProfile.gravityScale * ...
    t ^ styleProfile.nodeProfile.gravityExponent;
nodeProfile.bendSoftness = regionProfile.bendSoftnessScale * ...
    (styleProfile.nodeProfile.bendSoftnessRoot + ...
    (styleProfile.nodeProfile.bendSoftnessTip - ...
    styleProfile.nodeProfile.bendSoftnessRoot) * t);
end

function regionProfile = getRegionProfile(styleProfile, region)
regionName = char(string(region));
if isfield(styleProfile.regionProfiles, regionName)
    regionProfile = styleProfile.regionProfiles.(regionName);
else
    regionProfile = styleProfile.regionProfiles.scalp_top_left;
end
end
