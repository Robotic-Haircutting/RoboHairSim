function varargout = regionPlanCache(action, varargin)
%REGIONPLANCACHE Disk cache of Auto region plans under matlab_ur5e/config/plans.
%   One file per (method, region, target length), grouped by target length first:
%       config/plans/<len>cm/<method>/<region>.mat   (variables: regionPlan, meta)
%   method is "strict", "adaptive", "intralane" or "railslide" (the four
%   GUI-selectable planners), followed by "_<baseId>" for a non-fixed robot
%   base (e.g. "strict_rail135", "adaptive_ceiling").
%
%   file = regionPlanCache('path', configDir, method, region, targetLengthM)
%   regionPlanCache('save', configDir, method, regionPlan, context)
%   [plan, reason, file] = regionPlanCache('load', configDir, method, region, ...
%                                          targetLengthM, context)
%
%   context: struct with qHome, headCenterWorldM, R_world_head, baseId (default
%   "fixed") (what the plan
%   was made for). A cached plan is returned only when it still matches that
%   context, the hair it was planned for (activeNodeCounts, length/density
%   factors) and the planner source files it was made with; otherwise plan is
%   [] and reason says what differs, so the caller can offer a re-plan.
%   Nothing is re-planned or re-checked here: the plan is returned as saved.
switch string(action)
    case "path"
        varargout{1} = cachePath(varargin{:});
    case "save"
        saveCache(varargin{:});
    case "load"
        [varargout{1:3}] = loadCache(varargin{:});
    otherwise
        error("hairsim:BadCacheAction", "Unknown action: %s", action);
end
end

function file = cachePath(configDir, method, region, targetLengthM)
lenDir = sprintf('%scm', strrep(sprintf('%.2f', 100 * targetLengthM), '.', 'p'));
name = sprintf('%s.mat', region);
file = fullfile(configDir, 'plans', lenDir, char(method), name);
end

function saveCache(configDir, method, regionPlan, context)
file = cachePath(configDir, method, regionPlan.region, regionPlan.targetLengthM);
folder = fileparts(file);
if ~isfolder(folder)
    mkdir(folder);
end
meta = struct( ...
    'method', string(method), ...
    'baseId', contextBaseId(context), ...
    'qHome', context.qHome, ...
    'headCenterWorldM', context.headCenterWorldM, ...
    'R_world_head', context.R_world_head, ...
    'plannerStamp', plannerStamp(method), ...
    'saved', string(datetime('now')));
save(file, 'regionPlan', 'meta', '-v7.3');
end

function [plan, reason, file] = loadCache(configDir, method, region, targetLengthM, context)
plan = [];
file = cachePath(configDir, method, region, targetLengthM);
if ~isfile(file)
    reason = "no cached plan";
    return;
end
loaded = load(file, 'regionPlan', 'meta');
meta = loaded.meta;
if ~isequal(meta.plannerStamp, plannerStamp(method))
    reason = "planner code changed since it was cached";
elseif meta.baseId ~= contextBaseId(context)
    reason = "robot base placement differs";
elseif max(abs(meta.qHome(:) - context.qHome(:))) > 1e-9
    reason = "Home configuration differs";
elseif max(abs(meta.headCenterWorldM(:) - context.headCenterWorldM(:))) > 1e-9 || ...
        max(abs(meta.R_world_head(:) - context.R_world_head(:))) > 1e-9
    reason = "head pose differs";
else
    plan = loaded.regionPlan;
    reason = "";
end
end

function id = contextBaseId(context)
if isfield(context, 'baseId')
    id = string(context.baseId);
else
    id = "fixed";
end
end

function stamp = plannerStamp(method)
% Modification times of the planner files this method depends on. Any edit to
% them invalidates the cache, so a plan is never replayed from older logic.
%
% Updated for the HairSimApp core+strategy planning engine
% (+hairsim/+planning -- see docs/EXTENDING.md): a "method" name is now a
% named combination of 3 independent strategy files, not one monolithic
% planner file, so the stamp covers the shared engine plus whichever
% strategy files that preset actually uses (see
% hairsim.planning.registerBuiltinPresets for the name -> strategy mapping
% this must stay in sync with).
motionDir = fileparts(mfilename('fullpath'));
planningDir = fullfile(fileparts(motionDir), '+planning');
files = {fullfile(motionDir, 'solveRobotForClipperTip.m'), ...
    fullfile(planningDir, 'planRegionCore.m'), ...
    fullfile(planningDir, 'planOneLane.m'), ...
    fullfile(planningDir, '+orientationStrategies', 'strict.m'), ...
    fullfile(planningDir, '+laneRecoveryStrategies', 'failLane.m'), ...
    fullfile(planningDir, '+transitStrategies', 'direct.m')};
if startsWith(string(method), "adaptive")
    files{end + 1} = fullfile(planningDir, '+orientationStrategies', 'fallbackGrid.m');
elseif startsWith(string(method), "intralane")
    files{end + 1} = fullfile(planningDir, '+laneRecoveryStrategies', 'bridgeLift.m');
elseif startsWith(string(method), "railslide")
    files{end + 1} = fullfile(planningDir, '+transitStrategies', 'railSlide.m');
    files{end + 1} = fullfile(planningDir, 'buildLaneGeometry.m');
elseif startsWith(string(method), "camClipper")
    % the camClipper family plans in +cam/+post/+tool/+eval, not in the
    % strategy files above: any edit there must invalidate its plans too
    pkgDir = fileparts(motionDir);
    files{end + 1} = fullfile(planningDir, 'camClipperPlanner.m');
    files{end + 1} = fullfile(planningDir, 'camClipperTool.m');
    for sub = ["+cam", "+post", "+tool", "+eval"]
        listing = dir(fullfile(pkgDir, sub, '*.m'));
        files = [files, fullfile({listing.folder}, {listing.name})]; %#ok<AGROW>
    end
end
% a release build lacks the lane engine (planRegionCore & co.): only the
% files present can stamp a plan; in the full tree nothing is dropped
files = files(isfile(files));
stamp = zeros(1, numel(files));
for i = 1:numel(files)
    info = dir(files{i});
    stamp(i) = info.datenum;
end
end
