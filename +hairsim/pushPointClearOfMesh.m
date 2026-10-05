function pointOut = pushPointClearOfMesh( ...
        pointWorldM, meshWorld, toolClearanceRadiusM, requiredClearanceM, options)
%PUSHPOINTCLEAROFMESH Move a point along its own closest-mesh-face normal
%   until it clears requiredClearanceM of hairsim.evaluateClipperSphereMeshClearance's
%   surfaceClearanceM (using toolClearanceRadiusM). Re-evaluates the
%   closest face each iteration (rather than pushing once along the
%   original point's normal) so it stays correct as the point moves across
%   a curved surface, not just for small pushes.
%
%   Unlike hairsim.path.resolveSafeCuttingCenter, this has no upper bound
%   on how far it may push and no fixed search direction tied to a
%   cutting point's own scalp normal -- appropriate for a point with no
%   cutting semantics of its own (e.g. a default/rest tool-tip position),
%   which is free to sit anywhere comfortably clear of the mesh, unlike a
%   cutting-center candidate that must stay within the cutting sphere's
%   own reach of a specific hair point.

arguments
    pointWorldM (1, 3) double {mustBeFinite}
    meshWorld struct
    toolClearanceRadiusM (1, 1) double {mustBeNonnegative}
    requiredClearanceM (1, 1) double {mustBeNonnegative}
    options.MaxIterations (1, 1) double {mustBeInteger, mustBePositive} = 4
end

pointOut = pointWorldM;
for iter = 1:options.MaxIterations
    clearance = hairsim.evaluateClipperSphereMeshClearance( ...
        pointOut, meshWorld, toolClearanceRadiusM);
    if clearance.surfaceClearanceM >= requiredClearanceM
        return;
    end
    faceIdx = clearance.closestFaceIndex(1);
    localNormal = hairsim.normalizeVector(meshWorld.faceNormalsWorld(faceIdx, :), [1, 0, 0]);
    pushDistanceM = requiredClearanceM - clearance.surfaceClearanceM;
    candidateA = pointOut + localNormal * pushDistanceM;
    candidateB = pointOut - localNormal * pushDistanceM;
    clearanceA = hairsim.evaluateClipperSphereMeshClearance( ...
        candidateA, meshWorld, toolClearanceRadiusM);
    clearanceB = hairsim.evaluateClipperSphereMeshClearance( ...
        candidateB, meshWorld, toolClearanceRadiusM);
    if clearanceA.surfaceClearanceM >= clearanceB.surfaceClearanceM
        pointOut = candidateA;
    else
        pointOut = candidateB;
    end
end
end
