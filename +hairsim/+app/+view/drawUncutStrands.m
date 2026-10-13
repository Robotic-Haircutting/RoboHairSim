function handle = drawUncutStrands(ax, uncut, headCenterWorldM)
%DRAWUNCUTSTRANDS Mark the roots of the strands still to be cut
%   (hairsim.app.hair.uncutStrands) with red dots on the scalp, where long
%   hair cannot hide them. The dots sit 4 mm out from the root (away from
%   headCenterWorldM), above the region highlight. Tag 'HairSimTeleopUncut':
%   replaced on every call, survives the per-frame scene redraw.
%   uncut = [] removes them.

arguments
    ax
    uncut
    headCenterWorldM (1, 3) double = [NaN, NaN, NaN]
end

tag = 'HairSimTeleopUncut';
delete(findobj(ax, 'Tag', tag));
handle = gobjects(0);
if isempty(uncut) || isempty(uncut.idx)
    return;
end
r = uncut.rootsWorldM;
if all(isfinite(headCenterWorldM))
    out = r - headCenterWorldM;
    r = r + 0.004 * out ./ max(vecnorm(out, 2, 2), eps);
end
hold(ax, 'on');
handle = plot3(ax, r(:, 1), r(:, 2), r(:, 3), '.', 'Color', [0.90, 0.10, 0.10], 'MarkerSize', 14, 'Tag', tag);
end
