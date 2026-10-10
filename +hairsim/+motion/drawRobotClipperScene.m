function drawRobotClipperScene(ax, robot, qCurrent, headMesh, T_world_ee_actual, ...
        toolMount, cutFlag, solved, toolTipErrorM, options)
%DRAWROBOTCLIPPERSCENE Draw the UR5e, head mesh, and mounted clipper.
%   This function owns only the robot/head/tool visuals. Hair and debris
%   are a separate, independently updated overlay (see
%   hairsim.hair.render.updateHaircutLayerOverlay) so that redrawing the
%   robot pose never forces a hair/debris rebuild, and vice versa.

arguments
    ax
    robot
    qCurrent
    headMesh (1, 1) struct
    T_world_ee_actual (4, 4) double
    toolMount (1, 1) struct
    cutFlag (1, 1) logical
    solved (1, 1) logical
    toolTipErrorM (1, 1) double
    options.PoseStatus (1, 1) string = ""
    options.SceneZMaxM (1, 1) double {mustBePositive} = 0.65
    % Display clipper head (hairsim.motion.createClipperDisplayMesh); [] =
    % draw the parametric clipper model itself (toolMount.tool)
    options.ToolDisplayMesh = []
end

hasExistingView = ~isempty(get(ax, 'Children'));
if hasExistingView
    viewState = hairsim.captureAxesViewState(ax);
end

delete(findobj(ax, 'Tag', 'HairSimHeadMesh'));
delete(findobj(ax, 'Tag', 'HairSimClipperHead'));
if ~isempty(robot)
    hairsim.motion.showClipperRobot(robot, qCurrent, ax);
end
hold(ax, 'on');
patch(ax, ...
    'Faces', headMesh.faces, ...
    'Vertices', headMesh.vertices, ...
    'FaceColor', [0.88, 0.78, 0.68], ...
    'EdgeColor', 'none', ...
    'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.35, ...
    'DiffuseStrength', 0.70, ...
    'SpecularStrength', 0.08, ...
    'SpecularExponent', 8, ...
    'Tag', 'HairSimHeadMesh');

if cutFlag
    clipperColor = [0.1, 0.65, 0.25];
else
    clipperColor = [0.85, 0.08, 0.08];
end
if isempty(options.ToolDisplayMesh)
    drawClipperModel(ax, toolMount.tool, T_world_ee_actual, clipperColor);
else
    drawClipperHead(ax, options.ToolDisplayMesh, T_world_ee_actual, clipperColor);
end

axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
xlim(ax, [0.0, 0.85]);
ylim(ax, [-0.45, 0.45]);
zlim(ax, [0.0, options.SceneZMaxM]);
if hasExistingView
    hairsim.applyAxesViewState(ax, viewState);
else
    view(ax, [145, 25]);
end
hairsim.configureSceneLighting(ax);
if solved
    title(ax, '');
else
    titleText = sprintf('IK target rejected; holding previous pose (tool-tip error %.3f m)', ...
        toolTipErrorM);
    if strlength(options.PoseStatus) > 0
        titleText = sprintf('%s: %s', titleText, options.PoseStatus);
    end
    title(ax, titleText, 'Interpreter', 'none');
end
drawnow limitrate;
end

% =========================================================================

function drawClipperModel(ax, tool, T_world_ee, clipperColor)
%DRAWCLIPPERMODEL The parametric clipper (contact slab + body) as convex
%   hulls of its sample points, the contact slab colour-coded by cutFlag.
T_world_tcp = T_world_ee * tool.T_ee_tcp;
R = T_world_tcp(1:3, 1:3);
p = T_world_tcp(1:3, 4)';
pieces = {tool.contactPointsTcpM, clipperColor, 0.75; tool.bodyPointsTcpM, [0.32, 0.33, 0.36], 1.0};
for k = 1:size(pieces, 1)
    pts = pieces{k, 1} * R' + p;
    faces = convhulln(pieces{k, 1});
    patch(ax, 'Faces', faces, 'Vertices', pts, 'FaceColor', pieces{k, 2}, 'EdgeColor', 'none', ...
        'FaceAlpha', pieces{k, 3}, 'FaceLighting', 'gouraud', 'Tag', 'HairSimClipperHead');
end
end

function drawClipperHead(ax, displayMesh, T_world_ee, clipperColor)
%DRAWCLIPPERHEAD Clipper head wedge + flange adapter + colour-coded cutting face.
R = T_world_ee(1:3, 1:3);
p = T_world_ee(1:3, 4)';
toWorld = @(v) v * R' + p;
patch(ax, 'Faces', displayMesh.faces, 'Vertices', toWorld(displayMesh.verticesEe), ...
    'FaceColor', [0.32, 0.33, 0.36], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
    'AmbientStrength', 0.35, 'DiffuseStrength', 0.70, 'SpecularStrength', 0.35, ...
    'SpecularExponent', 15, 'Tag', 'HairSimClipperHead');
patch(ax, 'Faces', displayMesh.adapterFaces, 'Vertices', toWorld(displayMesh.adapterVerticesEe), ...
    'FaceColor', [0.55, 0.56, 0.58], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
    'Tag', 'HairSimClipperHead');
patch(ax, 'Faces', displayMesh.bladeFaces, 'Vertices', toWorld(displayMesh.bladeVerticesEe), ...
    'FaceColor', clipperColor, 'EdgeColor', clipperColor, 'LineWidth', 1.5, 'FaceAlpha', 0.6, ...
    'FaceLighting', 'none', 'Tag', 'HairSimClipperHead');
end
