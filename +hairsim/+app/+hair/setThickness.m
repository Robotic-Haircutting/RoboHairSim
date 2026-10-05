function appState = setThickness(appState, value)
%SETTHICKNESS Ported unchanged from run_robot_haircut.m's setHairThicknessControl.
%   No rebuild needed -- line width is a style-only property.
appState.hairLayer.style.lineWidth = value;
appState.hairLayer.haircutState.params.lineWidth = value;
end
