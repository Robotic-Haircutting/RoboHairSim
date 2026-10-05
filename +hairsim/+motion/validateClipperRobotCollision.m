function [collisionClear, rejectReason, sepDistM] = validateClipperRobotCollision( ...
        robot, qCandidate, endEffector, toolMount, headCollision, marginM, ...
        ikSolved, options)
%VALIDATECLIPPERROBOTCOLLISION Check the robot bodies against the head collision object.
%   headCollision may also be a CELL of collision objects (the head hull + environment
%   primitives, see createHeadCollisionModel and createEnvironmentCollisionPrimitives);
%   a single object behaves exactly as before.
%   The head object is the whole-head convex hull, a conservative safety envelope
%   for the ARM links (wrist included). It is not the scalp, so its distance must
%   not be used as a clearance criterion for the clipper sphere, which rides only
%   ~2 mm above the actual scalp. The attached ClipperTool_Link body is therefore
%   ALWAYS removed from this test (hairsim.motion.clearAttachedClipperCollision).
%   The clipper's safety criterion, d(clipper sphere, ACTUAL head mesh) >= d_min,
%   is enforced by the callers with hairsim.evaluateClipperSphereMeshClearance (the
%   planner's candidate and re-check gates, the teleop governor).
%
%   options.SkipClipperHeadCheck is kept for call-site compatibility only; it has
%   no effect.
%
%   sepDistM (optional 3rd output): the smallest separation between any robot
%   body and the head collision object.

arguments
    robot
    qCandidate
    endEffector
    toolMount
    headCollision
    marginM (1, 1) double
    ikSolved
    options.SkipClipperHeadCheck (1, 1) logical = false
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

robotForBodyCollision = hairsim.motion.clearAttachedClipperCollision(robot);
[robotClear, robotMinSepDist] = isRobotClearOfHead( ...
    robotForBodyCollision, qCandidate, headCollision, marginM);
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
