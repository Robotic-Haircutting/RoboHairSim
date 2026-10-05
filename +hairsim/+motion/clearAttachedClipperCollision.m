function robotWithoutClipperCollision = clearAttachedClipperCollision(robot)
%CLEARATTACHEDCLIPPERCOLLISION Return a copy of robot with the clipper's own
%   collision geometry removed, for checks that care only about the arm
%   body against the world (the clipper's own clearance to the head is
%   checked separately, e.g. by hairsim.evaluateClipperSphereMeshClearance).

clipperBodyName = 'ClipperTool_Link';
if ~any(strcmp(robot.BodyNames, clipperBodyName))
    robotWithoutClipperCollision = robot;
    return;
end
robotWithoutClipperCollision = copy(robot);
clipperBody = getBody(robotWithoutClipperCollision, clipperBodyName);
clearCollision(clipperBody);
end
