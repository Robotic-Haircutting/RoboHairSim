function handles = drawHaircutLayer(ax, haircutState, options)
%DRAWHAIRCUTLAYER Draw active hair and falling debris on an axes.

arguments
    ax (1, 1)
    haircutState (1, 1) struct
    options.DebrisColor (1, 3) double = [0.9, 0.05, 0.02]
    options.DebrisLineWidth (1, 1) double {mustBePositive} = 1.35
    options.MaxPointsPerLine (1, 1) double {mustBeInteger, mustBePositive} = 20000
end

hold(ax, 'on');
handles = struct();
% A line with more than 25000 points is drawn opaque: its alpha (the 4th
% Color element) is ignored. The hair is split at strand breaks (NaN) into
% lines below that, so it looks the same however much has been cut.
[X, Y, Z] = deal(haircutState.lineData.hairX, haircutState.lineData.hairY, haircutState.lineData.hairZ);
breaks = [0; find(isnan(X(:))); numel(X) + 1];
handles.hair = gobjects(0);
first = 1;
while first <= numel(X)
    last = breaks(find(breaks <= first - 1 + options.MaxPointsPerLine, 1, 'last'));
    if last < first                                  % one strand longer than the limit
        last = min(numel(X), first - 1 + options.MaxPointsPerLine);
    end
    handles.hair(end + 1) = plot3(ax, X(first:min(last, end)), Y(first:min(last, end)), Z(first:min(last, end)), ...
        '-', ...
        'Color', [haircutState.params.color, haircutState.params.alpha], ...
        'LineWidth', haircutState.params.lineWidth);
    first = last + 1;
end
if isempty(handles.hair)
    handles.hair = plot3(ax, nan, nan, nan, '-', ...
        'Color', [haircutState.params.color, haircutState.params.alpha], ...
        'LineWidth', haircutState.params.lineWidth);
end
handles.debris = plot3(ax, ...
    haircutState.lineData.debrisX, ...
    haircutState.lineData.debrisY, ...
    haircutState.lineData.debrisZ, ...
    '-', ...
    'Color', options.DebrisColor, ...
    'LineWidth', options.DebrisLineWidth);
uistack(handles.debris, 'top');
end
