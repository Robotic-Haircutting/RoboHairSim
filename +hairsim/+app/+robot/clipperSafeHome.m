function [qHome, parkCache] = clipperSafeHome(robotAppState, scene, parkCache)
%CLIPPERSAFEHOME The Home pose the interactive entries park the robot at.
%   The adapter's Home can leave the clipper blade inside the head at some
%   rail stations; this is the
%   clipper-safe park pose instead (hairsim.post.findClipperParkConfiguration:
%   Home pushed out from the head until the clipper clears it by 20 mm),
%   computed once per robot setup. Falls back to the adapter Home, with a
%   warning, when no such pose is found.
%
%   parkCache: struct from a previous call, or [] / struct() the first
%   time; it carries the poses per robot setup (basePlacementId + rail
%   angle) and the head distance query, so pass the returned one back in.

if isempty(parkCache) || ~isfield(parkCache, 'poses')
    parkCache = struct('poses', containers.Map('KeyType', 'char', 'ValueType', 'any'), 'headQuery', []);
end
key = sprintf('%s|%g', robotAppState.state.basePlacementId, robotAppState.state.railThetaRad);
if isKey(parkCache.poses, key)
    qHome = parkCache.poses(key);
    return;
end
qHome = robotAppState.modelContext.qHome(:);
try
    mc = robotAppState.modelContext;
    parkCtx = struct('robot', mc.robot, 'endEffector', mc.endEffector, 'toolMount', mc.toolMount, ...
        'headCollision', {mc.headCollision}, 'qHome', qHome, ...
        'headCenterWorldM', scene.headScene.headCenterWorldM, ...
        'headMeshSafetyWorld', scene.headScene.headMeshSafetyWorld, ...
        'redundantJointIndices', 1:numel(qHome), 'numJoints', numel(qHome));
    if isempty(parkCache.headQuery)
        parkCache.headQuery = hairsim.tool.createHeadDistanceQuery(scene.headScene.headMeshWorld);
    end
    qHome = hairsim.post.findClipperParkConfiguration(parkCtx, hairsim.tool.createClipperTool(), ...
        parkCache.headQuery, NaN);
catch parkError
    warning("hairsim:ClipperSafeHomeFailed", ...
        "No clipper-safe Home found (%s); using the adapter Home.", parkError.message);
end
parkCache.poses(key) = qHome;
end
