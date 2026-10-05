function handles = drawRegionPlanPreview(ax, regionPlan)
%DRAWREGIONPLANPREVIEW Static preview of a Milestone-2 region plan in ax.
%   Draws what hairsim.motion.planAutonomousRegionHaircut decided for the
%   WHOLE region at once, nothing else, and never moves the robot or touches
%   the hair. Three kinds of thing, plus the targets:
%     - scalp lane: thin dashed gray line, one per frozen lane;
%     - executable stroke: its safe CUT path (forward kinematics of the
%       planned joint path, 15 mm above the scalp) as a thick solid blue line;
%     - transit between two chained strokes (when transits were planned): orange
%       dashed path, and an executable stroke with no transit to it ("not
%       continuously reachable", not run) drawn in amber with a mark;
%     - infeasible stroke: its planned safe path as a red dotted line with a
%       red X at its middle (the robot could not follow it);
%     - target cut nodes of the region: green = within the tool radius of an
%       executable CUT path, gray = not covered by any executable stroke.
%   Each stroke carries its lane number; Run executes every executable stroke, in
%   lane order. Every object carries Tag 'HairSimRegionPlan', so a previous
%   preview is replaced on every call and survives the per-frame scene redraw
%   (which only deletes its own tags). The user's camera/limits are preserved.
%
%   The return value groups every drawn object by category so a caller can
%   toggle categories independently (e.g. hide infeasible strokes without
%   touching the lanes):
%     handles.lane            scalp lane lines (one per stroke)
%     handles.executable       executable stroke CUT paths (in-chain or not)
%     handles.infeasible       infeasible stroke paths + their X marks
%     handles.transit          feasible transit paths between chained strokes
%     handles.targetCovered    covered target-node scatter
%     handles.targetUncovered  uncovered target-node scatter
%     handles.label            per-stroke lane-number text labels
%   Every field is a gobjects column vector (possibly empty); all still carry
%   Tag 'HairSimRegionPlan', so deletion/replacement on the next call is
%   unchanged.

arguments
    ax
    regionPlan (1, 1) struct
end

tag = 'HairSimRegionPlan';
hasExistingView = ~isempty(get(ax, 'Children'));
if hasExistingView
    viewState = hairsim.captureAxesViewState(ax);
end
delete(findobj(ax, 'Tag', tag));
hold(ax, 'on');
handles = struct( ...
    'lane', gobjects(0, 1), ...
    'executable', gobjects(0, 1), ...
    'infeasible', gobjects(0, 1), ...
    'transit', gobjects(0, 1), ...
    'targetCovered', gobjects(0, 1), ...
    'targetUncovered', gobjects(0, 1), ...
    'label', gobjects(0, 1));

scalpStyle = {'Color', [0.45, 0.45, 0.45], 'LineStyle', '--', 'LineWidth', 0.8};
executableStyle = {'Color', [0.05, 0.30, 0.85], 'LineStyle', '-', 'LineWidth', 2.8};
infeasibleColor = [0.85, 0.10, 0.10];

targets = regionPlan.targetCutNodesWorldM;
covered = regionPlan.targetCovered(:);
if ~isempty(targets)
    if any(~covered)
        handles.targetUncovered(end + 1, 1) = scatter3(ax, targets(~covered, 1), ...
            targets(~covered, 2), targets(~covered, 3), 8, [0.35, 0.35, 0.35], ...
            'filled', 'Tag', tag);
    end
    if any(covered)
        handles.targetCovered(end + 1, 1) = scatter3(ax, targets(covered, 1), ...
            targets(covered, 2), targets(covered, 3), 12, [0.10, 0.65, 0.20], ...
            'filled', 'Tag', tag);
    end
end

for strokeIdx = 1:numel(regionPlan.strokes)
    stroke = regionPlan.strokes(strokeIdx);
    lane = regionPlan.allLanesWorldM{stroke.laneIdx};
    handles.lane(end + 1, 1) = plot3(ax, lane(:, 1), lane(:, 2), lane(:, 3), ...
        scalpStyle{:}, 'Tag', tag);

    label = sprintf('%d', stroke.laneIdx);
    inChain = ~regionPlan.transitsPlanned || any(regionPlan.chainStrokeIdx == strokeIdx);
    if stroke.executable
        path = stroke.achievedCutPathWorldM;
        style = executableStyle;
        if ~inChain
            style = {'Color', [0.90, 0.60, 0.05], 'LineStyle', '-', 'LineWidth', 2.8};
            label = [label ' (no transit)'];
        end
        handles.executable(end + 1, 1) = plot3(ax, path(:, 1), path(:, 2), path(:, 3), ...
            style{:}, 'Tag', tag);
        labelPoint = path(1, :);
        labelColor = [0.05, 0.20, 0.60];
    else
        path = stroke.safeCutPathWorldM;
        if size(path, 1) < 2
            path = lane;
        end
        handles.infeasible(end + 1, 1) = plot3(ax, path(:, 1), path(:, 2), path(:, 3), ...
            'Color', infeasibleColor, 'LineStyle', ':', 'LineWidth', 1.8, 'Tag', tag);
        middle = path(ceil(size(path, 1) / 2), :);
        handles.infeasible(end + 1, 1) = plot3(ax, middle(1), middle(2), middle(3), 'x', ...
            'Color', infeasibleColor, 'MarkerSize', 14, 'LineWidth', 3, 'Tag', tag);
        labelPoint = path(1, :);
        labelColor = infeasibleColor;
    end
    handles.label(end + 1, 1) = text(ax, labelPoint(1), labelPoint(2), labelPoint(3), ...
        ['  ' label], 'FontWeight', 'bold', 'FontSize', 9, 'Color', labelColor, ...
        'Tag', tag);
end

for transit = regionPlan.transits
    if transit.feasible
        path = transit.achievedPathWorldM;
        handles.transit(end + 1, 1) = plot3(ax, path(:, 1), path(:, 2), path(:, 3), ...
            'Color', [0.95, 0.45, 0.05], 'LineStyle', '--', 'LineWidth', 1.4, 'Tag', tag);
    end
end

if hasExistingView
    hairsim.applyAxesViewState(ax, viewState);
end
end
