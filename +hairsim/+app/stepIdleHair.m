function hairAppState = stepIdleHair(hairAppState, robotAppState, viewState, scene, floorZ)
%STEPIDLEHAIR One frame of hair life while the robot stands still: physics
%   and falling clippings advance, nothing is cut, and a comb-lift sweep in
%   progress is kept (the clipper is still where the last cut tick left
%   it). Without this the hair only moves on robot ticks, so clippings
%   freeze in mid-air whenever the arm stops (Teleop between key presses,
%   Auto when paused or finished). Not recorded: a recording is the robot's
%   trajectory. The interactive entries call it on frames without a
%   PoseCommand while clippings are falling or for a while after the last
%   robot tick / hair change, so the hair can settle.

mc = robotAppState.modelContext;
T_world_ee = getTransform(mc.robot, robotAppState.state.qCurrent, char(mc.endEffector));
[hairAppState, hairStepResult] = hairsim.app.hair.stepHairSim(hairAppState, T_world_ee, ...
    false, scene.headScene.headHullWorld, floorZ, 'SkipCutting', true);
if hairStepResult.shouldRender && ~isempty(viewState)
    hairsim.app.view.renderHairOverlay(viewState, hairAppState.hairLayer.haircutState);
end
end
