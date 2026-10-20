function appState = setThickness(appState, value)
%SETTHICKNESS Set the drawn hair thickness.
%   No rebuild needed -- line width is a style-only property.
appState.hairLayer.style.lineWidth = value;
appState.hairLayer.haircutState.params.lineWidth = value;
end
