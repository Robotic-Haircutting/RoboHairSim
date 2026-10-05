function pointWorldM = getMountedClipperPoint(T_world_ee, T_ee_point)
%GETMOUNTEDCLIPPERPOINT Transform a mounted clipper point into world space.

T_world_point = T_world_ee * T_ee_point;
pointWorldM = T_world_point(1:3, 4)';
end
