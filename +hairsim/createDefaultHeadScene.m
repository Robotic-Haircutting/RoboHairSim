function scene = createDefaultHeadScene(headStlPath)
%CREATEDEFAULTHEADSCENE Load the shared head mesh and its world pose.

arguments
    headStlPath
end

headMeshLocal = hairsim.loadHeadMeshMeters(headStlPath);
[T_world_head, headScaleFactor] = hairsim.createTrajectoryReplayHeadTransform();
R_world_head = T_world_head(1:3, 1:3);
headMeshWorld = hairsim.transformHeadMeshWithPose( ...
    headMeshLocal, T_world_head, 'MeshScaleFactor', headScaleFactor);

scene = struct();
scene.headMeshLocal = headMeshLocal;
scene.T_world_head = T_world_head;
scene.headScaleFactor = headScaleFactor;
scene.R_world_head = R_world_head;
scene.headMeshWorld = headMeshWorld;
scene.headCenterWorldM = headMeshWorld.center;
end
