function jump = emptyIntraLaneJump()
%EMPTYINTRALANEJUMP One intra-lane bridge attempt (see
%   hairsim.planning.laneRecoveryStrategies.bridgeLift), also reused as the
%   shape for a safety lift-off (planSafetyLiftOff-equivalent): fromCutIdx/
%   toCutIdx/numSkippedCutSamples are meaningless for a lift-off (left at 0).
jump = struct('fromCutIdx', 0, 'toCutIdx', 0, 'feasible', false, 'reason', "", ...
    'failedSample', 0, 'detail', "", 'samplePhase', strings(0, 1), ...
    'pointsWorldM', zeros(0, 3), 'rotations', zeros(3, 3, 0), 'qTrajectory', zeros(0, 0), ...
    'blendMaxJointDeg', nan, 'numBlendSamples', 0, 'numSkippedCutSamples', 0);
end
