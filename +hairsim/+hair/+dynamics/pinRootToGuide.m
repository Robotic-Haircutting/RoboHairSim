function hairState = pinRootToGuide(hairState, strandIdx)
%PINROOTTOGUIDE Snap a strand's root node to its guide target and zero its velocity.
%   Shared by hairsim.hair.dynamics.applyGuideHairHeadHullClearance and
%   hairsim.hair.dynamics.stepGuideHairPhysics.

hairState.hairs.X(1, strandIdx) = hairState.guideTargets.X(1, strandIdx);
hairState.hairs.Y(1, strandIdx) = hairState.guideTargets.Y(1, strandIdx);
hairState.hairs.Z(1, strandIdx) = hairState.guideTargets.Z(1, strandIdx);
hairState.vel.X(1, strandIdx) = 0;
hairState.vel.Y(1, strandIdx) = 0;
hairState.vel.Z(1, strandIdx) = 0;
end
