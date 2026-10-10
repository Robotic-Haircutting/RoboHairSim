function [collisionClear, rejectReason, sepDistM] = validateClipperRobotCollision( ...
        robot, qCandidate, endEffector, headCollision, marginM, ikSolved)
%VALIDATECLIPPERROBOTCOLLISION Check the robot links against the obstacles.
%   headCollision is one collision object or a CELL of them: the whole-head
%   convex hull (a conservative envelope for the arm links) plus, for a
%   floor-mounted base, the environment primitives (see
%   createHeadCollisionModel and createEnvironmentCollisionPrimitives).
%   robot is the arm without any tool body. The clipper's own clearance to
%   the exact head mesh is a separate gate
%   (hairsim.tool.evaluateClipperClearance), arm + clipper self collision
%   another (hairsim.post.createStationContext's armWithToolRobot).
%
%   sepDistM (optional 3rd output): the smallest separation between any robot
%   body and the obstacles.
arguments
    robot
    qCandidate
    endEffector %#ok<INUSA>
    headCollision
    marginM (1, 1) double
    ikSolved
end
collisionClear = true;
rejectReason = "OK";
sepDistM = inf;

if ~ikSolved
    collisionClear = false;
    rejectReason = "IK_ERROR";
    return;
end
if isempty(headCollision)
    return;
end

[robotClear, robotMinSepDist] = isRobotClearOfHead( ...
    robot, qCandidate, headCollision, marginM);
sepDistM = robotMinSepDist;
if ~robotClear
    collisionClear = false;
    rejectReason = "ROBOT_HEAD_COLLISION sep=" + sprintf("%.4f", robotMinSepDist);
end
end

function [isClear, minSepDist] = isRobotClearOfHead( ...
        robot, qCandidate, headCollision, marginM)
% headCollision is one collision object (the head) or a cell of objects (head
% plus environment primitives); the min separation is over all of them.
collisionObjects = headCollision;
if ~iscell(collisionObjects)
    collisionObjects = {collisionObjects};
end
[isColliding, sepDist] = checkCollision(robot, qCandidate, collisionObjects, ...
    'IgnoreSelfCollision', 'on');
finiteDistances = sepDist(isfinite(sepDist));
if isempty(finiteDistances)
    minSepDist = inf;
else
    minSepDist = min(finiteDistances(:));
end
isWorldColliding = any(isColliding(:));
isClear = ~isWorldColliding && minSepDist >= marginM;
end
