function headMeshWorld = transformHeadMeshWithPose(headMeshLocal, T_world_head, options)
%TRANSFORMHEADMESHWITHPOSE Transform raw STL-frame head vertices into world.

arguments
    headMeshLocal (1, 1) struct
    T_world_head (4, 4) double {mustBeFinite}
    options.MeshScaleFactor (1, 1) double {mustBePositive} = 1.0
end

verticesWorld = (headMeshLocal.vertices * options.MeshScaleFactor) * ...
    T_world_head(1:3, 1:3)' + T_world_head(1:3, 4)';
centeredFrameOriginWorldM = (headMeshLocal.center * options.MeshScaleFactor) * ...
    T_world_head(1:3, 1:3)' + T_world_head(1:3, 4)';

headMeshWorld = headMeshLocal;
headMeshWorld.vertices = verticesWorld;
headMeshWorld.center = mean(verticesWorld, 1);
headMeshWorld.centeredFrameOriginWorldM = centeredFrameOriginWorldM;
headMeshWorld.bboxMin = min(verticesWorld, [], 1);
headMeshWorld.bboxMax = max(verticesWorld, [], 1);
headMeshWorld.bboxSize = headMeshWorld.bboxMax - headMeshWorld.bboxMin;
headMeshWorld.R_world_head = T_world_head(1:3, 1:3);
headMeshWorld.T_world_head = T_world_head;
headMeshWorld.meshScaleFactor = options.MeshScaleFactor;
end
