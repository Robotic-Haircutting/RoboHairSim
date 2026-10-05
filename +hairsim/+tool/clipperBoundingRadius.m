function r = clipperBoundingRadius(tool, toolMount)
%CLIPPERBOUNDINGRADIUS Radius about the old clipper sphere's centre
%   (toolMount.T_ee_tool_center) that contains the clipper's contact slab
%   and wedge box. Sphere-based planners (e.g. hairsim.planning.
%   planSafeConfigurationTransition) are given this as toolClearanceRadiusM
%   so their clearance tests cover the real blade corners; the adapter
%   cylinder near the flange is left out (far from the head; the exact
%   clipper re-check covers it).
pts = [tool.contactPointsTcpM; tool.bodyPointsTcpM(1:45, :)] + [0, 0, tool.tcpOffsetM];
r = max(vecnorm(pts - toolMount.T_ee_tool_center(1:3, 4)', 2, 2));
end
