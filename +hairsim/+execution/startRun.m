function [executionState, segmentInfo] = startRun(executionState)
%STARTRUN Begin the loaded run at queue position 1. Call
%   hairsim.execution.loadPlan first (and SETUP-jump the robot to the
%   firstStroke it returns) before this.
if isempty(executionState.queue)
    error("hairsim:NoExecutionPlanLoaded", ...
        "hairsim.execution.loadPlan must be called before startRun.");
end
executionState.queuePos = 1;
executionState.strokesDone = 0;
executionState.isActive = true;
[executionState, segmentInfo] = loadSegmentAt(executionState, executionState.queuePos);
end
