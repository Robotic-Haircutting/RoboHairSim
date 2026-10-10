function toolMount = createClipperToolMount()
%CREATECLIPPERTOOLMOUNT The clipper mounted on the robot end effector.
%   The clipper is the parametric clipper model of
%   hairsim.tool.createClipperTool (contact slab + body + adapter); its
%   tool centre point (TCP, centre of the teeth face) sits on the flange's
%   +z axis. Every tool point the session uses -- IK targets, recorded tool
%   positions, clearance -- is this TCP.
%     toolMount.tool       the clipper model (geometry, comb-lift footprint)
%     toolMount.T_ee_tool  end effector -> TCP (= tool.T_ee_tcp)

tool = hairsim.tool.createClipperTool();
toolMount = struct();
toolMount.tool = tool;
toolMount.T_ee_tool = tool.T_ee_tcp;
toolMount.toolShape = "clipper";
end
