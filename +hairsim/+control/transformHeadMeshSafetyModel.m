function meshWorld = transformHeadMeshSafetyModel( ...
        meshLocal, headCenterWorldM, R_world_head, options)
%TRANSFORMHEADMESHSAFETYMODEL Move cached STL safety geometry into world.

arguments
    meshLocal struct
    headCenterWorldM (1, 3) double
    R_world_head (3, 3) double
    options.MeshScaleFactor (1, 1) double {mustBePositive} = 1.0
end

verticesWorld = (options.MeshScaleFactor * meshLocal.verticesLocal) * ...
    R_world_head' + headCenterWorldM;
faceCentersWorld = (options.MeshScaleFactor * meshLocal.faceCentersLocal) * ...
    R_world_head' + headCenterWorldM;
faceNormalsWorld = meshLocal.faceNormalsLocal * R_world_head';

meshWorld = struct();
meshWorld.verticesWorld = verticesWorld;
meshWorld.faces = meshLocal.faces;
meshWorld.faceNormalsWorld = faceNormalsWorld;
meshWorld.faceCentersWorld = faceCentersWorld;
meshWorld.headCenterWorldM = headCenterWorldM;
meshWorld.meshScaleFactor = options.MeshScaleFactor;
meshWorld.coordinateFrame = "world";
meshWorld.units = "meters";
meshWorld.sourcePath = meshLocal.sourcePath;
end
