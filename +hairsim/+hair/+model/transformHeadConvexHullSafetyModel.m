function hullWorld = transformHeadConvexHullSafetyModel( ...
        hullLocal, headCenterWorldM, R_world_head, options)
%TRANSFORMHEADCONVEXHULLSAFETYMODEL Move local hull planes into world frame.

arguments
    hullLocal struct
    headCenterWorldM (1, 3) double
    R_world_head (3, 3) double
    options.MeshScaleFactor (1, 1) double {mustBePositive} = 1.0
end

normalsWorld = hullLocal.normalsLocal * R_world_head';
offsetsWorldM = options.MeshScaleFactor * hullLocal.offsetsLocalM - ...
    normalsWorld * headCenterWorldM(:);

hullWorld = struct();
hullWorld.normalsWorld = normalsWorld;
hullWorld.offsetsWorldM = offsetsWorldM;
hullWorld.headCenterWorldM = headCenterWorldM;
hullWorld.meshScaleFactor = options.MeshScaleFactor;
hullWorld.coordinateFrame = "world";
hullWorld.units = "meters";
end
