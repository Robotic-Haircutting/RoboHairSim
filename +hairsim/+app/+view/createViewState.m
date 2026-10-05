function viewState = createViewState(ax, options)
%CREATEVIEWSTATE Visualization subsystem state: the axes handle (and,
%   optionally, the metrics text widget handle -- the App shell passes it
%   in once it has built the control panel; Visualization does not create
%   UI controls itself, only draws into handles it's given).
arguments
    ax
    options.MetricsTextHandle = []
end
viewState = struct('ax', ax, 'metricsTextHandle', options.MetricsTextHandle);
end
