function robotWithClipper = attachClipperCollisionToRobot(robot, endEffector, toolMount)
%ATTACHCLIPPERCOLLISIONTOROBOT Attach the clipper tool for collision checks.

arguments
    robot (1, 1) rigidBodyTree
    endEffector (1, 1) string
    toolMount (1, 1) struct
end

clipperBodyName = 'ClipperTool_Link';
if any(strcmp(robot.BodyNames, clipperBodyName))
    robotWithClipper = robot;
    return;
end

clipperBody = rigidBody(clipperBodyName);
clipperJoint = rigidBodyJoint('ClipperTool_Joint', 'fixed');
clipperBody.Joint = clipperJoint;
setFixedTransform(clipperJoint, toolMount.T_ee_tool_center);
addCollision(clipperBody, "sphere", toolMount.toolRadiusM);

addBody(robot, clipperBody, char(endEffector));
robotWithClipper = robot;
end
