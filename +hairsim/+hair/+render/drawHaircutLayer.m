function handles = drawHaircutLayer(ax, haircutState, options)
%DRAWHAIRCUTLAYER Draw active hair and falling debris on an axes.
%   CutHairColor: when set, strands that have been cut (lineData.cut*) are
%   drawn in that colour, thicker, with a dot at each root, so what has
%   been covered is visible even where the stubble is a few mm long;
%   handles.cut holds those objects. [] = all hair in the hair colour.

arguments
    ax (1, 1)
    haircutState (1, 1) struct
    options.DebrisColor (1, 3) double = [0.9, 0.05, 0.02]
    options.DebrisLineWidth (1, 1) double {mustBePositive} = 1.35
    options.MaxPointsPerLine (1, 1) double {mustBeInteger, mustBePositive} = 20000
    options.CutHairColor double = []
end

hold(ax, 'on');
handles = struct();
hairColor = [haircutState.params.color, haircutState.params.alpha];
lineData = haircutState.lineData;
handles.cut = gobjects(0);
if isempty(options.CutHairColor)
    handles.hair = drawSplit(lineData.hairX, lineData.hairY, lineData.hairZ, hairColor, haircutState.params.lineWidth);
else
    handles.hair = drawSplit(lineData.uncutX, lineData.uncutY, lineData.uncutZ, hairColor, haircutState.params.lineWidth);
    handles.cut = drawSplit(lineData.cutX, lineData.cutY, lineData.cutZ, options.CutHairColor, ...
        2 * haircutState.params.lineWidth);
    rootIdx = find(~isnan(lineData.cutX(:)) & [true; isnan(lineData.cutX(1:end - 1))]);
    if ~isempty(rootIdx)
        handles.cut(end + 1) = plot3(ax, lineData.cutX(rootIdx), lineData.cutY(rootIdx), lineData.cutZ(rootIdx), ...
            '.', 'Color', options.CutHairColor, 'MarkerSize', 9);
    end
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

    function h = drawSplit(X, Y, Z, color, lineWidth)
        % A line with more than 25000 points is drawn opaque: its alpha (the
        % 4th Color element) is ignored. The hair is split at strand breaks
        % (NaN) into lines below that, so it looks the same however much
        % has been cut.
        breaks = [0; find(isnan(X(:))); numel(X) + 1];
        h = gobjects(0);
        first = 1;
        while first <= numel(X)
            last = breaks(find(breaks <= first - 1 + options.MaxPointsPerLine, 1, 'last'));
            if last < first                              % one strand longer than the limit
                last = min(numel(X), first - 1 + options.MaxPointsPerLine);
            end
            h(end + 1) = plot3(ax, X(first:min(last, end)), Y(first:min(last, end)), Z(first:min(last, end)), ...
                '-', 'Color', color, 'LineWidth', lineWidth); %#ok<AGROW>
            first = last + 1;
        end
    end
end
