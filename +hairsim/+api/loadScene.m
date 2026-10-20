function scene = loadScene(options)
%LOADSCENE Load the head mesh, hair-root model and planning data this
%   library needs, as a single library call -- no GUI required.
%
%   Data come from data/ (or, if there is none, a data/ one level up):
%   HeadScan.stl (head mesh), hair_roots.json (one entry per root, with its
%   scalp region) and HairClipper.stl (clipper mesh). To use your own head,
%   put the same three files in a folder and pass it:
%   scene = hairsim.api.loadScene('DataDir', '/path/to/your/data')
%   (file formats: the "Use your own head and hair roots" part of
%   README.md).
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
paths.clipperDisplayStl = fullfile(dataDir, 'HairClipper.stl');
paths.clipperTrajectoryCsv = fullfile(outputsDir, 'clipper_trajectory.csv');

headScene = hairsim.createDefaultHeadScene(paths.headStl);
hairLayer = hairsim.hair.createDefaultHairLayer(paths.hairRoots, ...
    headScene.headMeshLocal, headScene.T_world_head, headScene.headScaleFactor);

scene = struct();
scene.paths = paths;
scene.rootDir = rootDir;
scene.headScene = headScene;
scene.hairLayer = hairLayer;
scene.haircutState = hairLayer.haircutState;
end
