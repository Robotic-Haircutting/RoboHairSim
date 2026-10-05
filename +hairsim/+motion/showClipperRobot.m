function showClipperRobot(robot, qCurrent, ax)
%SHOWCLIPPERROBOT Render UR5e visual meshes without rebuilding each frame.
%   show(...,'FastUpdate',true) only reuses the plot of the SAME
%   rigidBodyTree handle. A base-placement change (rail station slide,
%   switchBase) hands in a NEW robot object; show() then does a full draw,
%   and since the axes is held and PreservePlot=false only removes the new
%   object's own previous plot, the old robot's meshes would stay behind as
%   ghosts. So the drawn handles are tracked here and the old robot is
%   deleted explicitly whenever the robot object changes.

lastRobot = [];
if isappdata(ax, 'hairsimRobotVisualSource')
    lastRobot = getappdata(ax, 'hairsimRobotVisualSource');
end
visualHandles = gobjects(0);
if isappdata(ax, 'hairsimRobotVisualHandles')
    visualHandles = getappdata(ax, 'hairsimRobotVisualHandles');
end
useFastUpdate = ~isempty(lastRobot) && isvalid(lastRobot) && lastRobot == robot && ...
    ~isempty(visualHandles) && all(isgraphics(visualHandles));

if useFastUpdate
    try
        show(robot, qCurrent, ...
            'Parent', ax, ...
            'PreservePlot', false, ...
            'Frames', 'off', ...
            'Visuals', 'on', ...
            'Collisions', 'off', ...
            'FastUpdate', true);
        return;
    catch
        % isgraphics(visualHandles) above can pass yet the handles still
        % turn out unusable to FastUpdate -- e.g. "Invalid or deleted
        % object" deep inside
        % robotics.manip.internal.RigidBodyTreeVisualizationHelper,
        % observed over a long replay in MATLAB desktop's docked
        % "Figures" panel. Fall through to a full rebuild below instead
        % of letting this crash kill the whole replay; that path already
        % deletes any stale visualHandles and re-primes the cache.
        useFastUpdate = false;
    end
end
if ~useFastUpdate
    delete(visualHandles(isgraphics(visualHandles)));
    childrenBefore = allchild(ax);
    show(robot, qCurrent, ...
        'Parent', ax, ...
        'PreservePlot', false, ...
        'Frames', 'off', ...
        'Visuals', 'on', ...
        'Collisions', 'off');
    childrenAfter = allchild(ax);
    setappdata(ax, 'hairsimRobotVisualHandles', ...
        childrenAfter(~ismember(childrenAfter, childrenBefore)));
    setappdata(ax, 'hairsimRobotVisualSource', robot);
end
end
