function renderScene(viewState, renderRequest)
%RENDERSCENE Draw the robot/head/clipper scene. Thin wrapper around
%   hairsim.motion.drawRobotClipperScene -- RenderRequest's fields are
%   exactly that function's argument list, named instead of positional.
%
%   renderRequest fields: robot, qCurrent, headMeshWorld, T_world_ee_actual,
%   toolMount, cutFlag, solved, toolTipErrorM, poseStatus, sceneZMaxM, and
%   optionally toolDisplayMesh (clipper head mesh,
%   hairsim.motion.createClipperDisplayMesh; [] or absent = the parametric
%   clipper model).
arguments
    viewState (1, 1) struct
    renderRequest (1, 1) struct
end
toolDisplayMesh = [];
if isfield(renderRequest, 'toolDisplayMesh')
    toolDisplayMesh = renderRequest.toolDisplayMesh;
end
hairsim.motion.drawRobotClipperScene(viewState.ax, renderRequest.robot, ...
    renderRequest.qCurrent, renderRequest.headMeshWorld, renderRequest.T_world_ee_actual, ...
    renderRequest.toolMount, renderRequest.cutFlag, renderRequest.solved, renderRequest.toolTipErrorM, ...
    'PoseStatus', string(renderRequest.poseStatus), 'SceneZMaxM', renderRequest.sceneZMaxM, ...
    'ToolDisplayMesh', toolDisplayMesh);
end
