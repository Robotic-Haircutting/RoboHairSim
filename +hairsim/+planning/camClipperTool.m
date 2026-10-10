function [tool, isCamClipper] = camClipperTool(plannerName)
%CAMCLIPPERTOOL The clipper tool a camClipper-family planner preset plans
%   with -- and so the tool its plans must be cut with (comb-lift cut
%   model) when executed or replayed. One place, so planning, GUI
%   execution, full demo and bundle replay cannot disagree.
%
%     "camClipper"            hairsim.tool.createClipperTool() (feed
%                             direction ignored; the 2026-10-03 results)
%     "camClipperAgainstFlow" same tool, comb lifts only hair it is pushed
%                             against: feed within 45 deg of -hair flow
%     "geometricRaster"       the hair-unaware baseline: plans without the
%                             hair flow, but is judged and cut by the SAME
%                             tool as camClipperAgainstFlow (45 deg rule)
%
%   isCamClipper: plannerName belongs to the camClipper family (tool is []
%   otherwise).

arguments
    plannerName (1, 1) string
end

isCamClipper = true;
switch plannerName
    case "camClipper"
        tool = hairsim.tool.createClipperTool();
    case {"camClipperAgainstFlow", "geometricRaster"}
        tool = hairsim.tool.createClipperTool('MaxFeedAngleRad', deg2rad(45));
    otherwise
        tool = [];
        isCamClipper = false;
end
end
