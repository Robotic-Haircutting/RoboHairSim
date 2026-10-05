function out = regionPlannerStore(action, name, plannerFn)
%REGIONPLANNERSTORE Package-private persistent registry of named region
%   planners. Not part of the public API; use
%   hairsim.planning.registerRegionPlanner and
%   hairsim.planning.getRegionPlanner instead.

persistent registry
if isempty(registry)
    registry = containers.Map('KeyType', 'char', 'ValueType', 'any');
end

switch action
    case "register"
        registry(char(name)) = plannerFn;
        out = [];
    case "get"
        key = char(name);
        if ~isKey(registry, key)
            error("hairsim:UnknownRegionPlanner", ...
                "No region planner registered for '%s'. Registered: %s", ...
                key, strjoin(keys(registry), ', '));
        end
        out = registry(key);
    case "names"
        out = string(keys(registry));
    otherwise
        error("hairsim:InvalidRegionPlannerStoreAction", "Unknown action '%s'.", action);
end
end
