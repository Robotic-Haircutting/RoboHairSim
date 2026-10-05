function runPaths = createRobotHaircutRunPaths(rootDir, runId)
%CREATEROBOTHAIRCUTRUNPATHS Create output paths for one robot haircut run.

if nargin < 2 || strlength(string(runId)) == 0
    runId = char("robot_haircut_" + string(datetime('now', 'Format', "yyyyMMdd'T'HHmmss")));
end

runDir = fullfile(rootDir, 'outputs', 'runs', char(runId));
if ~isfolder(runDir)
    mkdir(runDir);
end

runPaths = struct();
runPaths.root = rootDir;
runPaths.runId = char(runId);
runPaths.runDir = runDir;
runPaths.configJson = fullfile(runDir, 'config.json');
runPaths.metricsJson = fullfile(runDir, 'metrics.json');
runPaths.summaryTxt = fullfile(runDir, 'summary.txt');
runPaths.clipperTrajectoryCsv = fullfile(runDir, 'clipper_trajectory.csv');
runPaths.robotJointTrajectoryCsv = fullfile(runDir, 'robot_joint_trajectory.csv');
runPaths.ur5eCartesianCsv = fullfile(runDir, 'ur5e_cartesian_trajectory.csv');
runPaths.poseLogCsv = fullfile(runDir, 'pose_log.csv');
runPaths.cutEventsCsv = fullfile(runDir, 'cut_events.csv');
runPaths.autoPlanSummaryCsv = fullfile(runDir, 'auto_plan_summary.csv');
end
