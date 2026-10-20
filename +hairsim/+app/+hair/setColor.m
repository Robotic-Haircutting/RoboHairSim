function appState = setColor(appState, newColor)
%SETCOLOR Set the hair colour.
%   No rebuild needed -- color is a style-only property.
appState.hairLayer.style.color = newColor;
appState.hairLayer.haircutState.params.color = newColor;
end
