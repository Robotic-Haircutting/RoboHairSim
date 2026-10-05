function scene = createDefaultHeadScene(headStlPath, headConvexHullJsonPath, options)
%CREATEDEFAULTHEADSCENE Load the shared head mesh, pose, and safety models.

arguments
    headStlPath
    headConvexHullJsonPath
    options.HeadMeshSafetyJsonPath = ""
end

headMeshLocal = hairsim.loadHeadMeshMeters(headStlPath);
[T_world_head, headScaleFactor] = hairsim.createTrajectoryReplayHeadTransform();
R_world_head = T_world_head(1:3, 1:3);
headMeshWorld = hairsim.transformHeadMeshWithPose( ...
    headMeshLocal, T_world_head, 'MeshScaleFactor', headScaleFactor);
[headHullLocal, hullCacheCreated] = hairsim.hair.model.createHeadConvexHullSafetyModel( ...
    headMeshLocal, headConvexHullJsonPath);
headHullWorld = hairsim.hair.model.transformHeadConvexHullSafetyModel( ...
    headHullLocal, headMeshWorld.centeredFrameOriginWorldM, R_world_head, ...
    'MeshScaleFactor', headScaleFactor);
headMeshSafetyLocal = [];
headMeshSafetyWorld = [];
meshSafetyCacheCreated = false;
if strlength(string(options.HeadMeshSafetyJsonPath)) > 0
    [headMeshSafetyLocal, meshSafetyCacheCreated] = ...
        hairsim.control.createHeadMeshSafetyModel( ...
            headMeshLocal, options.HeadMeshSafetyJsonPath);
    headMeshSafetyWorld = hairsim.control.transformHeadMeshSafetyModel( ...
        headMeshSafetyLocal, headMeshWorld.centeredFrameOriginWorldM, R_world_head, ...
        'MeshScaleFactor', headScaleFactor);
end

scene = struct();
scene.headMeshLocal = headMeshLocal;
scene.T_world_head = T_world_head;
scene.headScaleFactor = headScaleFactor;
scene.R_world_head = R_world_head;
scene.headMeshWorld = headMeshWorld;
scene.headCenterWorldM = headMeshWorld.center;
% headHull*: hair-physics proxy only, never for robot-head or tool-head collision.
scene.headHullLocal = headHullLocal;
scene.headHullWorld = headHullWorld;
scene.hullCacheCreated = hullCacheCreated;
scene.headMeshSafetyLocal = headMeshSafetyLocal;
scene.headMeshSafetyWorld = headMeshSafetyWorld;
scene.meshSafetyCacheCreated = meshSafetyCacheCreated;
end
