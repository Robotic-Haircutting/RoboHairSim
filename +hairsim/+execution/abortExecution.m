function executionState = abortExecution(executionState)
%ABORTEXECUTION Stop at once: nothing more is replayed. Session/UI reset is
%   not done here -- that belongs to Recording/the App shell, which should
%   call hairsim.app.recording.endSession and update its own UI state
%   alongside calling this.
executionState.trajectoryIndex = 0;
executionState.isActive = false;
executionState.queue = struct('kind', {}, 'index', {});
executionState.queuePos = 0;
executionState.strokesDone = 0;
executionState.currentSegment = [];
end
