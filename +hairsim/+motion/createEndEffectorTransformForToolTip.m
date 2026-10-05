function T_world_ee = createEndEffectorTransformForToolTip( ...
        toolTipWorldM, R_world_ee, T_ee_tool)
%CREATEENDEFFECTORTRANSFORMFORTOOLTIP Preserve a requested clipper tip point.

eePositionWorldM = toolTipWorldM(:) - R_world_ee * T_ee_tool(1:3, 4);

T_world_ee = eye(4);
T_world_ee(1:3, 1:3) = R_world_ee;
T_world_ee(1:3, 4) = eePositionWorldM;
end
