function planningState = createAutoPlanningState(configDir)
%CREATEAUTOPLANNINGSTATE Auto Planning subsystem state: just the last plan
%   result and where the on-disk plan cache lives. Deliberately does not
%   hold a reference to the robot/scene it plans against -- those are
%   passed explicitly into hairsim.app.planning.planHaircut's PlanRequest
%   each call.
arguments
    configDir (1, 1) string
end
planningState = struct('configDir', configDir, 'lastPlan', [], 'lastPlanProvenance', []);
end
