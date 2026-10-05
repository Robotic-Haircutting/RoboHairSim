function config = createRobotHaircutConfig(paths, runPaths, options)
%CREATEROBOTHAIRCUTCONFIG Create reproducible robot haircut run metadata.

arguments
    paths (1, 1) struct
    runPaths (1, 1) struct
    options.Mode (1, 1) string = "interactive"
    options.MaxToolTipErrorM (1, 1) double {mustBeNonnegative} = 0.03
    options.MaxJointStepRad (1, 1) double {mustBeNonnegative} = 1.0
    options.CollisionMarginM (1, 1) double {mustBeNonnegative} = 0
    options.TargetHairLengthM (1, 1) double = nan
    options.TargetHairLengthToleranceM (1, 1) double = nan
    options.ToolMount = []
end

config = struct();
config.project = "HairSim";
config.stage = "robot_haircut_teleop_auto";
config.mode = options.Mode;
config.runId = runPaths.runId;
config.headStl = paths.headStl;
config.hairRoots = paths.hairRoots;
config.headConvexHullJson = paths.headConvexHullJson;
config.headMeshSafetyJson = paths.headMeshSafetyJson;
config.maxToolTipErrorM = options.MaxToolTipErrorM;
config.maxJointStepRad = options.MaxJointStepRad;
config.collisionMarginM = options.CollisionMarginM;
config.targetHairLengthM = options.TargetHairLengthM;
config.targetHairLengthToleranceM = options.TargetHairLengthToleranceM;
if isstruct(options.ToolMount)
    config.tool = createToolConfig(options.ToolMount);
end
config.outputs = struct( ...
    'clipperTrajectoryCsv', runPaths.clipperTrajectoryCsv, ...
    'robotJointTrajectoryCsv', runPaths.robotJointTrajectoryCsv, ...
    'ur5eCartesianCsv', runPaths.ur5eCartesianCsv, ...
    'poseLogCsv', runPaths.poseLogCsv, ...
    'cutEventsCsv', runPaths.cutEventsCsv, ...
    'autoPlanSummaryCsv', runPaths.autoPlanSummaryCsv);
end

function toolConfig = createToolConfig(toolMount)
toolConfig = struct();
copyFields = ["toolShape", "toolRadiusM", "toolClearanceRadiusM"];
for fieldName = copyFields
    if isfield(toolMount, fieldName)
        toolConfig.(fieldName) = toolMount.(fieldName);
    end
end
end
