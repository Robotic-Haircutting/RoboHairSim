function limits = revoluteJointLimits(robot)
%REVOLUTEJOINTLIMITS PositionLimits (k x 2) of the robot's revolute joints,
%   in body order. Reads robot.Bodies once: every read of that property
%   copies all bodies of the tree, so callers in a loop should keep this
%   result (see hairsim.post.createStationContext).

bodies = robot.Bodies;
limits = zeros(0, 2);
for bodyIdx = 1:numel(bodies)
    joint = bodies{bodyIdx}.Joint;
    if strcmp(joint.Type, 'revolute')
        limits(end + 1, :) = joint.PositionLimits; %#ok<AGROW>
    end
end
end
