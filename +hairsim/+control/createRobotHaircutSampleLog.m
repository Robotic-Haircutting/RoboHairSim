function sampleLog = createRobotHaircutSampleLog(jointCount)
%CREATEROBOTHAIRCUTSAMPLELOG Create an empty per-sample robot haircut log.

sampleLog = struct();
sampleLog.time = zeros(0, 1);
sampleLog.mode = strings(0, 1);
sampleLog.status = strings(0, 1);
sampleLog.solved = false(0, 1);
sampleLog.cutFlag = false(0, 1);
sampleLog.governorActive = false(0, 1);
sampleLog.toolTipErrorM = zeros(0, 1);
sampleLog.cutCount = zeros(0, 1);
sampleLog.q = zeros(0, jointCount);
sampleLog.targetTipWorldM = zeros(0, 3);
sampleLog.desiredTipWorldM = zeros(0, 3);
sampleLog.safeTipWorldM = zeros(0, 3);
sampleLog.actualTipWorldM = zeros(0, 3);
end
