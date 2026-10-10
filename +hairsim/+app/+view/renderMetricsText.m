function renderMetricsText(viewState, phase, metrics, options)
%RENDERMETRICSTEXT Format and display the metrics text widget. Ported from
%   run_robot_haircut.m's updateLiveMetricsText/updateIdleMetricsText/
%   updateSavedMetricsText -- moved here (not Recording) because setting a
%   widget's 'String' is a view concern; the numbers themselves come from
%   hairsim.app.recording.queryMetrics (phase "live"/"idle") or
%   hairsim.app.recording.finalizeSession's result.metrics (phase "saved").
%
%   phase == "live":  metrics = queryMetrics(recordingState, "live")
%   phase == "idle":  metrics = queryMetrics(recordingState, "idle")
%   phase == "saved": metrics = finalizeSession's result.metrics;
%                      options.AutoRunSummary optionally prefixes the
%                      headline (matches the original's autoRunSummary).

arguments
    viewState (1, 1) struct
    phase (1, 1) string {mustBeMember(phase, ["live", "idle", "saved"])}
    metrics (1, 1) struct
    options.AutoRunSummary (1, 1) string = ""
    options.RunId (1, 1) string = ""
end

if isempty(viewState.metricsTextHandle) || ~ishandle(viewState.metricsTextHandle)
    return;
end

switch phase
    case "live"
        text = sprintf(['Status: recording\n', ...
            'Samples: %d\n', ...
            'Cut events: %d'], metrics.numSamples, metrics.numCutEvents);
    case "idle"
        text = sprintf(['Status: idle\n', ...
            'Run: %s\n', ...
            'Ready to start'], metrics.runId);
    otherwise % "saved"
        if isfield(metrics, 'targetLengthMeanAbsErrorM')
            if strlength(options.AutoRunSummary) > 0
                headline = sprintf('Done. %s', options.AutoRunSummary);
            else
                headline = 'Done.';
            end
            text = sprintf(['%s\n', ...
                'Cut %d strands, removed %.1f%%\n', ...
                'Length MAE %.1f mm, overcut %d\n', ...
                'Saved %s'], ...
                headline, metrics.cutStrandCount, 100 * metrics.removedRatio, ...
                1000 * metrics.targetLengthMeanAbsErrorM, metrics.targetLengthOvercutCount, ...
                options.RunId);
        else
            text = sprintf(['Saved: %s\n', ...
                'Removed: %.3f\n', ...
                'Events: %d\n', ...
                'Min clearance: %.4f m'], ...
                options.RunId, metrics.removedRatio, metrics.totalCutEvents, metrics.minClipperClearanceM);
        end
end
set(viewState.metricsTextHandle, 'String', text);
end
