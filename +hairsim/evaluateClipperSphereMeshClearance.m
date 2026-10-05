function clearance = evaluateClipperSphereMeshClearance( ...
        clipperCentersWorldM, meshWorld, toolRadiusM)
%EVALUATECLIPPERSPHEREMESHCLEARANCE Measure center and surface mesh clearance.

clipperCentersWorldM = double(clipperCentersWorldM);
numSamples = size(clipperCentersWorldM, 1);
centerClearanceM = nan(numSamples, 1);
surfaceClearanceM = nan(numSamples, 1);
closestFaceIndex = nan(numSamples, 1);
closestPointWorldM = nan(numSamples, 3);

if numSamples == 0 || isempty(meshWorld)
    clearance = createResult(centerClearanceM, surfaceClearanceM, ...
        closestFaceIndex, closestPointWorldM, toolRadiusM);
    return;
end

for sampleIdx = 1:numSamples
    [closestPoint, faceIndex, distanceM] = hairsim.closestPointOnMesh( ...
        clipperCentersWorldM(sampleIdx, :), meshWorld);
    centerClearanceM(sampleIdx) = distanceM;
    surfaceClearanceM(sampleIdx) = distanceM - toolRadiusM;
    closestFaceIndex(sampleIdx) = faceIndex;
    closestPointWorldM(sampleIdx, :) = closestPoint;
end

clearance = createResult(centerClearanceM, surfaceClearanceM, ...
    closestFaceIndex, closestPointWorldM, toolRadiusM);
end

function clearance = createResult(centerClearanceM, surfaceClearanceM, ...
        closestFaceIndex, closestPointWorldM, toolRadiusM)
clearance = struct();
clearance.toolRadiusM = toolRadiusM;
clearance.centerClearanceM = centerClearanceM;
clearance.surfaceClearanceM = surfaceClearanceM;
clearance.closestFaceIndex = closestFaceIndex;
clearance.closestPointWorldM = closestPointWorldM;
clearance.hasHeadPenetration = any(surfaceClearanceM < 0);
clearance.headPenetrationSampleCount = nnz(surfaceClearanceM < 0);
clearance.minCenterClearanceM = minFiniteOrNan(centerClearanceM);
clearance.minSurfaceClearanceM = minFiniteOrNan(surfaceClearanceM);
end

function value = minFiniteOrNan(values)
values = values(isfinite(values));
if isempty(values)
    value = nan;
else
    value = min(values);
end
end

