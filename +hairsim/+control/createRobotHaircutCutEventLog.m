function cutEvents = createRobotHaircutCutEventLog()
%CREATEROBOTHAIRCUTCUTEVENTLOG Create an empty discrete haircut-event log.

cutEvents = struct();
cutEvents.sampleIndex = zeros(0, 1);
cutEvents.time = zeros(0, 1);
cutEvents.mode = strings(0, 1);
cutEvents.strandIndex = zeros(0, 1);
cutEvents.nodeIndex = zeros(0, 1);
end
