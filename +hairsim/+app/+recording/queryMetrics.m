function metrics = queryMetrics(recordingState, phase)
%QUERYMETRICS Pure-data summary numbers for the "live" or "idle" phase (the
%   "saved"/finalized phase's numbers are hairsim.app.recording.
%   finalizeSession's own result.metrics -- there is nothing further to
%   query once a session is finalized). Returns a struct of NUMBERS ONLY;
%   turning it into displayed text is hairsim.app.view.renderMetricsText's
%   job, not this function's.
arguments
    recordingState (1, 1) struct
    phase (1, 1) string {mustBeMember(phase, ["live", "idle"])}
end
if phase == "live"
    metrics = struct( ...
        'numSamples', numel(recordingState.sampleLog.time), ...
        'numCutEvents', numel(recordingState.cutEvents.sampleIndex));
else
    metrics = struct('runId', string(recordingState.runPaths.runId));
end
end
