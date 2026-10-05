function out = robotAdapterStore(action, name, builderFn)
%ROBOTADAPTERSTORE Package-private persistent registry of robot adapter builders.
%   Not part of the public API; use hairsim.robot.registerRobotAdapter and
%   hairsim.robot.getRobotAdapterBuilder instead. action is "register",
%   "get" or "names".

persistent registry
if isempty(registry)
    registry = containers.Map('KeyType', 'char', 'ValueType', 'any');
end

switch action
    case "register"
        registry(char(name)) = builderFn;
        out = [];
    case "get"
        key = char(name);
        if ~isKey(registry, key)
            error("hairsim:UnknownRobotAdapter", ...
                "No robot adapter registered for '%s'. Registered: %s", ...
                key, strjoin(keys(registry), ', '));
        end
        out = registry(key);
    case "names"
        out = string(keys(registry));
    otherwise
        error("hairsim:InvalidRobotAdapterStoreAction", "Unknown action '%s'.", action);
end
end
