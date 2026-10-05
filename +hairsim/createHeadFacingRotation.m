function R_world_ee = createHeadFacingRotation(toolTipWorldM, headCenterWorldM)
%CREATEHEADFACINGROTATION Point end-effector local +Z toward the head.

toolAxisWorld = headCenterWorldM(:) - toolTipWorldM(:);
toolAxisWorld = hairsim.normalizeVector(toolAxisWorld, [1; 0; 0]);

upAxisWorld = [0; 0; 1];
toolYAxisWorld = upAxisWorld - dot(upAxisWorld, toolAxisWorld) * toolAxisWorld;
if norm(toolYAxisWorld) < 1e-8
    upAxisWorld = [0; 1; 0];
    toolYAxisWorld = upAxisWorld - dot(upAxisWorld, toolAxisWorld) * toolAxisWorld;
end
toolYAxisWorld = hairsim.normalizeVector(toolYAxisWorld, [0; 0; 1]);

toolXAxisWorld = cross(toolYAxisWorld, toolAxisWorld);
toolXAxisWorld = hairsim.normalizeVector(toolXAxisWorld, [0; 1; 0]);
toolYAxisWorld = cross(toolAxisWorld, toolXAxisWorld);
toolYAxisWorld = hairsim.normalizeVector(toolYAxisWorld, [0; 0; 1]);

R_world_ee = [toolXAxisWorld, toolYAxisWorld, toolAxisWorld];
end
