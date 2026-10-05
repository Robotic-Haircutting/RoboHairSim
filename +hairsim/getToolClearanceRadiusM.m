function radiusM = getToolClearanceRadiusM(toolMount)
%GETTOOLCLEARANCERADIUSM Resolve the clipper clearance radius from a tool mount.
%   Prefers toolMount.toolClearanceRadiusM when present, falling back to
%   toolMount.toolRadiusM. Shared by hairsim.control.finalizeRobotHaircutRun
%   and hairsim.motion.summarizeAutoStrokePlan.

if isfield(toolMount, 'toolClearanceRadiusM')
    radiusM = toolMount.toolClearanceRadiusM;
else
    radiusM = toolMount.toolRadiusM;
end
end
