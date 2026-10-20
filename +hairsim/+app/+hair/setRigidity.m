function appState = setRigidity(appState, value)
%SETRIGIDITY Set the hair rigidity.
%   No rebuild needed -- rigidity is read by the physics step every frame.
appState.hairLayer.physics.rigidity = value;
appState.hairLayer.haircutState.params.rigidity = value;
end
