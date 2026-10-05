function executionState = abortExecution(executionState)
%ABORTEXECUTION Stop at once: nothing more is replayed. Ported from
%   run_robot_haircut.m's abortRegionPlanRun (minus the session/UI-reset
%   side effects -- those belong to Recording/the App shell, which should
%   call hairsim.app.recording.endSession and update its own UI state
%   alongside calling this).
executionState.trajectoryIndex = 0;
executionState.isActive = false;
executionState.queue = struct('kind', {}, 'index', {});
executionState.queuePos = 0;
executionState.strokesDone = 0;
executionState.currentSegment = [];
end
