function builderFn = getRobotAdapterBuilder(name)
%GETROBOTADAPTERBUILDER Look up a registered robot adapter builder by name.
%   Most callers want hairsim.robot.createRobotAdapter instead, which calls
%   this internally and then runs the shared bootstrap. This accessor
%   exists for introspection/testing.

arguments
    name (1, 1) string
end
ensureBuiltinRobotAdaptersRegistered();
builderFn = robotAdapterStore("get", name);
end
