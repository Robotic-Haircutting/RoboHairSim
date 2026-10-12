function viewState = createViewState(ax, options)
%CREATEVIEWSTATE Visualization subsystem state: the axes handle (and,
%   optionally, the metrics text widget handle -- the App shell passes it
%   in once it has built the control panel; Visualization does not create
%   UI controls itself, only draws into handles it's given).
%   CutHairColor: draw strands that have been cut in this colour, with a
%   dot at each root, so the covered scalp shows (interactive entries);
%   [] = every strand in the hair colour (videos, replays).
arguments
    ax
    options.MetricsTextHandle = []
    options.CutHairColor double = []
end
viewState = struct('ax', ax, 'metricsTextHandle', options.MetricsTextHandle, ...
    'cutHairColor', options.CutHairColor);
end
