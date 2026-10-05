function vector = normalizeVector(vector, defaultVector)
%NORMALIZEVECTOR Normalize a vector, falling back to defaultVector near zero.
%   Shared by hairsim.createHeadFacingRotation, hairsim.createSurfaceFollowingRotation,
%   hairsim.control.createHeadMotionFrame, and several hairsim.hair.model.* files,
%   which previously each carried their own copy of this helper. Always
%   re-derives lengthValue from the (possibly just-substituted) vector and
%   divides by max(lengthValue, eps), so a non-unit defaultVector is itself
%   normalized on the fallback path -- preserve this when reusing it, some
%   callers rely on it (e.g. hairsim.hair.model.evaluateHairStrandNodes
%   passes non-unit defaults like baseFlow/worldGravity).

lengthValue = norm(vector);
if lengthValue < eps
    vector = defaultVector;
    lengthValue = norm(vector);
end
vector = vector / max(lengthValue, eps);
end
