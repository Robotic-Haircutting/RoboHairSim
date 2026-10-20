function [tool, isCamClipper] = camClipperTool(plannerName)
%CAMCLIPPERTOOL The clipper tool a camClipper-family planner plans with --
%   and so the tool its plans must be cut with (comb-lift cut model) when
%   executed or replayed. One place, so planning, GUI execution, full demo
%   and bundle replay cannot disagree. The camClipper family is exactly
%   the planner names listed here:
%
%     "camClipperAgainstFlow" hairsim.tool.createClipperTool with the feed
%                             rule: the comb lifts only hair it is pushed
%                             against, feed within 45 deg of -hair flow
%     "geometricRaster"       the hair-unaware baseline (registered by the
%                             benchmark code, not in a release): plans
%                             without the hair flow, but is judged and cut
%                             by the SAME tool as camClipperAgainstFlow
%
%   isCamClipper: plannerName belongs to the camClipper family (tool is []
%   otherwise). A planner of your own that should be cut with the clipper
%   model is added here.

arguments
    plannerName (1, 1) string
end

isCamClipper = true;
switch plannerName
    case {"camClipperAgainstFlow", "geometricRaster"}
        tool = hairsim.tool.createClipperTool('MaxFeedAngleRad', deg2rad(45));
    otherwise
        tool = [];
        isCamClipper = false;
end
end
