function ensureBuiltinRobotAdaptersRegistered()
%ENSUREBUILTINROBOTADAPTERSREGISTERED Lazily register the shipped adapters.
%   Idempotent: registering the same name twice just overwrites the
%   builder with an identical one, which is harmless.

persistent done
if isequal(done, true)
    return;
end
hairsim.robot.registerRobotAdapter("ur5e", @hairsim.robot.adapters.ur5e);
hairsim.robot.registerRobotAdapter("gen3", @hairsim.robot.adapters.gen3);
done = true;
end
