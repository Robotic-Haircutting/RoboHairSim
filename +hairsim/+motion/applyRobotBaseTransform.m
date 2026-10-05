function placedRobot = applyRobotBaseTransform(robot, T_world_base)
%APPLYROBOTBASETRANSFORM Copy of robot rigidly moved by T_world_base.
%   Every body whose parent is the tree root gets
%       JointToParent := T_world_base * JointToParent(nominal),
%   so the URDF's own root->child transforms (universalUR5e's are Rz(pi)) are
%   kept and the whole arm moves as one rigid body: for any q and any body,
%       getTransform(placed, q, b) == T_world_base * getTransform(robot, q, b).
%   Joint count, order and limits are unchanged (q stays 6-D). Apply this to
%   the robot BEFORE the clipper tool is attached or after -- the tool hangs
%   off tool0, not the root, so either order is equivalent.

arguments
    robot (1, 1) rigidBodyTree
    T_world_base (4, 4) double {mustBeFinite}
end
placedRobot = copy(robot);
rootName = placedRobot.BaseName;
numMoved = 0;
for bodyIdx = 1:placedRobot.NumBodies
    body = placedRobot.Bodies{bodyIdx};
    if strcmp(body.Parent.Name, rootName)
        T_nominal = body.Joint.JointToParentTransform;
        setFixedTransform(body.Joint, T_world_base * T_nominal);
        numMoved = numMoved + 1;
    end
end
assert(numMoved >= 1, "hairsim:NoRootChildren", "Robot has no body attached to its root.");
end
