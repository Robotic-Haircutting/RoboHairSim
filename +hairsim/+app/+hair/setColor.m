function appState = setColor(appState, newColor)
%SETCOLOR Ported unchanged from run_robot_haircut.m's setHairColorControl.
%   No rebuild needed -- color is a style-only property.
appState.hairLayer.style.color = newColor;
appState.hairLayer.haircutState.params.color = newColor;
end
