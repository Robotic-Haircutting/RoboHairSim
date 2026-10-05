function [sampleLog, cutEvents] = appendRobotHaircutSample( ...
        sampleLog, cutEvents, sample)
%APPENDROBOTHAIRCUTSAMPLE Append one pose sample and its cut events.

sampleIndex = numel(sampleLog.time) + 1;
sampleLog.time(end + 1, 1) = double(sample.time);
sampleLog.mode(end + 1, 1) = string(sample.mode);
sampleLog.status(end + 1, 1) = string(sample.status);
sampleLog.solved(end + 1, 1) = logical(sample.solved);
sampleLog.cutFlag(end + 1, 1) = logical(sample.cutFlag);
sampleLog.governorActive(end + 1, 1) = logical(sample.governorActive);
sampleLog.toolTipErrorM(end + 1, 1) = double(sample.toolTipErrorM);
sampleLog.cutCount(end + 1, 1) = double(sample.cutInfo.cutCount);
sampleLog.q(end + 1, :) = sample.q(:)';
sampleLog.targetTipWorldM(end + 1, :) = sample.targetTipWorldM;
sampleLog.desiredTipWorldM(end + 1, :) = sample.desiredTipWorldM;
sampleLog.safeTipWorldM(end + 1, :) = sample.safeTipWorldM;
sampleLog.actualTipWorldM(end + 1, :) = sample.actualTipWorldM;
sampleLog.clipperCenterWorldM(end + 1, :) = sample.clipperCenterWorldM;

for cutIdx = 1:sample.cutInfo.cutCount
    cutEvents.sampleIndex(end + 1, 1) = sampleIndex;
    cutEvents.time(end + 1, 1) = double(sample.time);
    cutEvents.mode(end + 1, 1) = string(sample.mode);
    cutEvents.strandIndex(end + 1, 1) = sample.cutInfo.strandIndices(cutIdx);
    cutEvents.nodeIndex(end + 1, 1) = sample.cutInfo.nodeIndices(cutIdx);
end
end
