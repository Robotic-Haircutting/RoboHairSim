function toolTipWorldM = createInitialClipperToolTip( ...
        headCenterWorldM, headMeshSafetyWorld, toolMount, options)
%CREATEINITIALCLIPPERTOOLTIP Return the default clipper start point.
%   options.StartTipWorldM is a fixed empirical constant near the head,
%   not derived from the scalp geometry -- under the corrected clearance
%   check (toolClearanceRadiusM = toolRadiusM instead of the old 0), that
%   literal constant sits ~1-2mm INSIDE the required safety boundary. That
%   alone was enough to deadlock an entire auto session: every
%   stroke's trajectory validation
%   (the since-removed waypoint planner's validateJointTrajectory)
%   checks the shared starting sample too, the robot's position never
%   advances after a failed attempt, so one marginal starting point
%   rejects 100% of subsequent attempts regardless of their own target or
%   path. Pushed outward (via hairsim.pushPointClearOfMesh, following the
%   local scalp-mesh normal, not a fixed direction) to a safe clearance
%   before being returned, with an extra options.DRobustM margin beyond
%   the bare feasibility boundary: unlike a cutting-center candidate
%   (hairsim.path.resolveSafeCuttingCenter), this point has no cutting
%   semantics forcing it close to the surface, so it can afford to sit
%   comfortably clear instead of right at the boundary.
%   headMeshSafetyWorld/toolMount are optional so callers without a real
%   mesh safety model (e.g. minimal synthetic test scenes) still get the
%   literal constant back unmodified, matching this function's
%   pre-existing behavior for them.

arguments
    headCenterWorldM (1, 3) double %#ok<INUSA>
    headMeshSafetyWorld struct = struct()
    toolMount struct = struct()
    options.StartTipWorldM (1, 3) double = [0.4850, -0.0200, 0.2520]
    options.DSafeM (1, 1) double {mustBeNonnegative} = 0.001
    options.DRobustM (1, 1) double {mustBeNonnegative} = 0.002
end

toolTipWorldM = options.StartTipWorldM;
if isfield(headMeshSafetyWorld, 'faces') && isfield(toolMount, 'toolClearanceRadiusM')
    requiredClearanceM = options.DSafeM + options.DRobustM;
    toolTipWorldM = hairsim.pushPointClearOfMesh( ...
        toolTipWorldM, headMeshSafetyWorld, toolMount.toolClearanceRadiusM, requiredClearanceM);
end
end
