function strokePlan = createStrokePlan( ...
        startTipWorldM, targetTipWorldM, T_world_ee_target, ...
        qStart, qTarget, qTrajectory, replayTipTargetsWorldM, status, ...
        desiredTipTargetsWorldM, desiredRotationsWorldEe, samplePhase, transitionType)
%CREATESTROKEPLAN Build a replayable stroke-plan record from a joint trajectory.
%   status labels how qTrajectory was produced (e.g. "hybrid_joint_path" for
%   an IK/RRT waypoint stroke) and is stored verbatim in
%   strokePlan.plan.status. Used by
%   hairsim.motion.planAutonomousRegionHaircut.
%   desiredTipTargetsWorldM/desiredRotationsWorldEe/samplePhase/
%   transitionType (all optional, diagnostic-only): the planner's own
%   per-sample INTENDED pose and phase, captured verbatim by the caller
%   (planAutonomousRegionHaircut passes its primitive's own values)
%   rather than reconstructed after the fact --
%   nothing here recomputes or alters them. Not read by replay or any
%   production consumer of strokePlan; exists so an offline analysis pass
%   can compare achieved (qTrajectory, via forward kinematics) against
%   intended without re-deriving planner geometry. When omitted (older
%   callers), these default to NaN/"UNSPECIFIED" placeholders sized to
%   match qTrajectory, preserving the N_desired == N_qTrajectory
%   invariant every consumer of these fields relies on.

sampleCount = size(qTrajectory, 2);
if nargin < 9
    desiredTipTargetsWorldM = nan(sampleCount, 3);
end
if nargin < 10
    desiredRotationsWorldEe = nan(3, 3, sampleCount);
end
if nargin < 11
    samplePhase = repmat("UNSPECIFIED", sampleCount, 1);
end
if nargin < 12
    transitionType = "unspecified";
end
timestamp = 0:(sampleCount - 1);
strokePlan = struct();
strokePlan.startTipWorldM = startTipWorldM;
strokePlan.targetTipWorldM = targetTipWorldM;
strokePlan.replayTipTargetsWorldM = replayTipTargetsWorldM;
strokePlan.T_world_ee_start = nan(4, 4);
strokePlan.T_world_ee_target = T_world_ee_target;
strokePlan.qStart = qStart;
strokePlan.poseFinal = [T_world_ee_target(1:3, 4); ...
    rotm2eul(T_world_ee_target(1:3, 1:3), 'XYZ')'];
strokePlan.qTarget = qTarget;
strokePlan.controller = [];
strokePlan.plan = struct( ...
    'positions', qTrajectory, ...
    'velocities', zeros(size(qTrajectory)), ...
    'accelerations', zeros(size(qTrajectory, 1), max(sampleCount - 1, 1)), ...
    'timestamp', timestamp, ...
    'status', status, ...
    'lastExitFlag', 1, ...
    'lastInfo', [], ...
    'desiredTipTargetsWorldM', desiredTipTargetsWorldM, ...
    'desiredRotationsWorldEe', desiredRotationsWorldEe, ...
    'samplePhase', samplePhase, ...
    'transitionType', transitionType);
end
