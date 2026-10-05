function clearance = evaluateClipperClearance(T_world_tcp, tool, headQuery, options)
%EVALUATECLIPPERCLEARANCE Clipper-vs-head clearance at one or more TCP poses.
%   T_world_tcp: (4,4) or (4,4,N). For each pose, the contact slab's and the
%   body's sample points (hairsim.tool.createClipperTool) are placed in the
%   world and their exact signed distance to the head mesh is taken
%   (hairsim.tool.signedDistanceToHead). Unlike the old sphere test this
%   depends on the tool's ORIENTATION: rolling or pitching the blade moves
%   its corners toward or away from the scalp.
%
%   clearance fields (N x 1 each): contactMinM, bodyMinM, minM (the smaller),
%   ok (minM >= tool.safetyClearanceM), worstPointWorldM (N x 3).

arguments
    T_world_tcp (4, 4, :) double
    tool (1, 1) struct
    headQuery (1, 1) struct
    options.CapM (1, 1) double {mustBePositive} = 0.03
end

numPoses = size(T_world_tcp, 3);
numContact = size(tool.contactPointsTcpM, 1);
localPoints = [tool.contactPointsTcpM; tool.bodyPointsTcpM];
contactMinM = nan(numPoses, 1);
bodyMinM = nan(numPoses, 1);
worstPointWorldM = nan(numPoses, 3);
for k = 1:numPoses
    R = T_world_tcp(1:3, 1:3, k);
    t = T_world_tcp(1:3, 4, k)';
    worldPoints = localPoints * R' + t;
    d = hairsim.tool.signedDistanceToHead(worldPoints, headQuery, options.CapM);
    contactMinM(k) = min(d(1:numContact));
    bodyMinM(k) = min(d(numContact + 1:end));
    [~, worst] = min(d);
    worstPointWorldM(k, :) = worldPoints(worst, :);
end
minM = min(contactMinM, bodyMinM);
clearance = struct('contactMinM', contactMinM, 'bodyMinM', bodyMinM, 'minM', minM, ...
    'ok', minM >= tool.safetyClearanceM, 'worstPointWorldM', worstPointWorldM);
end
