function handles = updateHaircutLayerOverlay(ax, haircutState, options)
%UPDATEHAIRCUTLAYEROVERLAY Redraw the hair/debris overlay on its own.
%   Manages the 'HairSimSceneOverlay' tagged graphics independently of the
%   robot/head scene in hairsim.motion.drawRobotClipperScene, so hair and
%   debris only redraw when hair state actually changes, and the robot
%   pose only redraws when the robot actually moves.

arguments
    ax (1, 1)
    haircutState (1, 1) struct
    options.DebrisColor (1, 3) double = [0.9, 0.05, 0.02]
    options.DebrisLineWidth (1, 1) double {mustBePositive} = 1.35
    options.CutHairColor double = []     % hairsim.hair.render.drawHaircutLayer
end

delete(findobj(ax, 'Tag', 'HairSimSceneOverlay'));
handles = hairsim.hair.render.drawHaircutLayer(ax, haircutState, ...
    'DebrisColor', options.DebrisColor, ...
    'DebrisLineWidth', options.DebrisLineWidth, 'CutHairColor', options.CutHairColor);
set([handles.hair(:); handles.cut(:); handles.debris], 'Tag', 'HairSimSceneOverlay');
drawnow limitrate;
end
