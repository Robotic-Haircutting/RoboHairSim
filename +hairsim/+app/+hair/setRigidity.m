function appState = setRigidity(appState, value)
%SETRIGIDITY Ported unchanged from run_robot_haircut.m's setHairRigidityControl.
%   No rebuild needed -- rigidity is read by the physics step every frame.
appState.hairLayer.physics.rigidity = value;
appState.hairLayer.haircutState.params.rigidity = value;
end
