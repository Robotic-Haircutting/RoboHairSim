function toolMount = createClipperToolMount()
%CREATECLIPPERTOOLMOUNT Return the simulated clipper mount on the robot EE.
%   The clipper head is modeled as a single sphere, mounted toolTipOffsetM
%   along the end effector's local +Z axis. toolRadiusM is that sphere's
%   physical/cutting radius. toolClearanceRadiusM is the radius the exact
%   scalp-mesh safety check (hairsim.evaluateClipperSphereMeshClearance,
%   hairsim.control.applyMountedClipperMeshSafetyGovernor) subtracts from
%   the measured center-to-mesh distance -- it must be AT LEAST toolRadiusM,
%   otherwise that check is only protecting a point at the sphere's center,
%   not the sphere's own physical extent, and a pose it accepts as having
%   +1mm margin can in fact have the real 13mm ball penetrating the scalp
%   mesh by ~12mm. Kept as its own field (rather than reusing toolRadiusM
%   directly at each call site) so it can later be set larger than
%   toolRadiusM if the collision approximation needs its own margin beyond
%   the physical tool radius.

toolDiameterM = 0.026;
toolRadiusM = 0.5 * toolDiameterM;
toolClearanceRadiusM = toolRadiusM;
% Sits the clipper sphere flush against the UR5e tool0 flange face (no
% mount rod modeled) -- empirically calibrated so the sphere's near
% surface touches the flange's real visual mesh instead of floating in
% front of it or sinking into it.
toolTipOffsetM = 0.032;

T_ee_tool_center = eye(4);
T_ee_tool_center(1:3, 4) = [0; 0; toolTipOffsetM];

T_ee_tool = eye(4);
T_ee_tool(1:3, 4) = [0; 0; toolTipOffsetM];

toolMount = struct();
toolMount.T_ee_tool = T_ee_tool;
toolMount.T_ee_tool_center = T_ee_tool_center;
toolMount.toolRadiusM = toolRadiusM;
toolMount.toolClearanceRadiusM = toolClearanceRadiusM;
toolMount.toolCenterOffsetFromTipM = norm( ...
    T_ee_tool(1:3, 4) - T_ee_tool_center(1:3, 4));
toolMount.toolShape = "sphere";
end
