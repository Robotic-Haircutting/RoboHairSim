function robot = createArmWithClipperBody(armRobot, endEffector, tool, qHome)
%CREATEARMWITHCLIPPERBODY A copy of the arm with the clipper body attached,
%   for the arm + clipper SELF-collision test: a box over the wedge's upper
%   part and the adapter cylinder of hairsim.tool.createClipperTool (the
%   cylinder starts 10 mm off the flange so it never touches the wrist link
%   it is bolted to). The clipper's clearance to the head is NOT tested with
%   this body but with the exact clipper model
%   (hairsim.tool.evaluateClipperClearance).

robot = copy(armRobot);
body = rigidBody('ClipperBody_Link');
setFixedTransform(body.Joint, eye(4));
boxZ = tool.tcpOffsetM + mean(tool.bodyPointsTcpM(1:45, 3));
boxSize = [2 * max(tool.bodyPointsTcpM(1:45, 1)), 2 * max(tool.bodyPointsTcpM(1:45, 2)), ...
    max(tool.bodyPointsTcpM(1:45, 3)) - min(tool.bodyPointsTcpM(1:45, 3))];
addCollision(body, "box", boxSize, trvec2tform([0, 0, boxZ]));
cylTop = tool.tcpOffsetM + min(tool.bodyPointsTcpM(1:45, 3));
cylLen = cylTop - 0.010;
addCollision(body, "cylinder", [0.008, cylLen], trvec2tform([0, 0, 0.010 + cylLen / 2]));
addBody(robot, body, char(endEffector));
if checkCollision(robot, qHome(:), 'SkippedSelfCollisions', 'parent')
    warning("hairsim:post:ToolBodySelfCollisionAtHome", ...
        "Clipper body self-collides at Home; self-collision falls back to the arm only.");
    robot = copy(armRobot);
end
end
