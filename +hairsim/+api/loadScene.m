function scene = loadScene(options)
%LOADSCENE Load the head mesh, hair-root model and planning data this
%   library needs, as a single library call -- no GUI required.
%
%   Unlike the original matlab_ur5e bootstrap (which hardcoded data/ as a
%   sibling of the matlab folder), every path here is overridable, so you
%   can point this at your own head-scan data instead of the bundled
%   placeholder in HairSimApp/data: scene = hairsim.api.loadScene(
%   'DataDir', '/path/to/your/data') uses HeadScan.stl, hair_roots.json,
%   HeadScan_convex_hull.json, HeadScan_mesh_safety.json and
%   head_regions.json from that folder instead (same file names and
%   formats the bundled placeholder uses -- see docs/EXTENDING.md).
%
%   options.PlanningDataDir is the root the sessions read config/ from and
%   write runs to (scene.rootDir); it defaults to this app's own root.
%
%   scene fields: paths, headScene (hairsim.createDefaultHeadScene's
%   result), hairLayer, haircutState (= hairLayer.haircutState, the usual
%   starting point for hairsim.api.planRegion), rootDir (=
%   options.PlanningDataDir, resolved).

arguments
    options.DataDir (1, 1) string = ""
    options.OutputsDir (1, 1) string = ""
    options.PlanningDataDir (1, 1) string = ""
end

appRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));

dataDir = options.DataDir;
if strlength(dataDir) == 0
    % the app's own data/, else a data/ shared one level up (the
    % development tree keeps the head scan next to, not inside, the app)
    dataDir = fullfile(appRoot, 'data');
    sharedDataDir = fullfile(fileparts(appRoot), 'data');
    if ~isfolder(dataDir) && isfolder(sharedDataDir)
        dataDir = sharedDataDir;
    end
end
outputsDir = options.OutputsDir;
if strlength(outputsDir) == 0
    outputsDir = fullfile(appRoot, 'outputs');
end
rootDir = options.PlanningDataDir;
if strlength(rootDir) == 0
    rootDir = appRoot;
end

paths = struct();
paths.root = rootDir;
paths.data = dataDir;
paths.outputs = outputsDir;
paths.headStl = fullfile(dataDir, 'HeadScan.stl');
paths.hairRoots = fullfile(dataDir, 'hair_roots.json');
paths.headConvexHullJson = fullfile(dataDir, 'HeadScan_convex_hull.json');
paths.headMeshSafetyJson = fullfile(dataDir, 'HeadScan_mesh_safety.json');
paths.clipperDisplayStl = fullfile(dataDir, 'HairClipper.stl');
paths.clipperTrajectoryCsv = fullfile(outputsDir, 'clipper_trajectory.csv');

headScene = hairsim.createDefaultHeadScene(paths.headStl, paths.headConvexHullJson, ...
    'HeadMeshSafetyJsonPath', paths.headMeshSafetyJson);
hairLayer = hairsim.hair.createDefaultHairLayer(paths.hairRoots, ...
    headScene.headMeshLocal, headScene.T_world_head, headScene.headScaleFactor);

scene = struct();
scene.paths = paths;
scene.rootDir = rootDir;
scene.headScene = headScene;
scene.hairLayer = hairLayer;
scene.haircutState = hairLayer.haircutState;
end
