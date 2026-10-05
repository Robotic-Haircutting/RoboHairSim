function renderScene(viewState, renderRequest)
%RENDERSCENE Draw the robot/head/clipper scene. Thin wrapper around
%   hairsim.motion.drawRobotClipperScene -- RenderRequest's fields are
%   exactly that function's argument list, named instead of positional.
%
%   renderRequest fields: robot, qCurrent, headMeshWorld, T_world_ee_actual,
%   T_ee_tool, T_ee_tool_center, toolRadiusM, cutFlag, solved,
%   toolTipErrorM, poseStatus, sceneZMaxM, and optionally toolDisplayMesh
%   (display-only clipper head, hairsim.motion.createClipperDisplayMesh;
%   [] or absent = the original sphere).
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
    renderRequest.T_ee_tool, renderRequest.T_ee_tool_center, renderRequest.toolRadiusM, ...
    renderRequest.cutFlag, renderRequest.solved, renderRequest.toolTipErrorM, ...
    'PoseStatus', string(renderRequest.poseStatus), 'SceneZMaxM', renderRequest.sceneZMaxM, ...
    'ToolDisplayMesh', toolDisplayMesh);
end
