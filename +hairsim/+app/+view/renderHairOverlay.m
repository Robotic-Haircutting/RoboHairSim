function renderHairOverlay(viewState, haircutState)
%RENDERHAIROVERLAY Redraw the hair/debris overlay. Thin wrapper around
%   hairsim.hair.render.updateHaircutLayerOverlay; independent of
%   renderScene so hair-only changes never force a robot redraw and vice
%   versa.
arguments
    viewState (1, 1) struct
    haircutState (1, 1) struct
end
cutHairColor = [];
if isfield(viewState, 'cutHairColor')
    cutHairColor = viewState.cutHairColor;
end
hairsim.hair.render.updateHaircutLayerOverlay(viewState.ax, haircutState, 'CutHairColor', cutHairColor);
end
