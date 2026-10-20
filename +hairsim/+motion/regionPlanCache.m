function varargout = regionPlanCache(action, varargin)
%REGIONPLANCACHE Disk cache of region plans under <configDir>/plans (config/plans).
%   One file per (method, region, target length), grouped by target length first:
%       config/plans/<len>cm/<method>/<region>.mat   (variables: regionPlan, meta)
%   method is a registered planner name (hairsim.planning.registerBuiltinPresets),
%   followed by "_<baseId>" for a non-fixed robot base (e.g.
%   "camClipperAgainstFlow_rail135").
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
% For a camClipper-family method that is the planner, its tool, the
% transit connector and every file of +cam, +post, +tool and +eval, plus
% the IK helper every method uses.
motionDir = fileparts(mfilename('fullpath'));
planningDir = fullfile(fileparts(motionDir), '+planning');
files = {fullfile(motionDir, 'solveRobotForClipperTip.m')};
if isCamClipperMethod(method)
    % the camClipper family plans in +cam/+post/+tool/+eval: any edit there
    % must invalidate its plans too
    pkgDir = fileparts(motionDir);
    files{end + 1} = fullfile(planningDir, 'camClipperPlanner.m');
    files{end + 1} = fullfile(planningDir, 'camClipperTool.m');
    files{end + 1} = fullfile(planningDir, 'planSafeConfigurationTransition.m');
    for sub = ["+cam", "+post", "+tool", "+eval"]
        listing = dir(fullfile(pkgDir, sub, '*.m'));
        files = [files, fullfile({listing.folder}, {listing.name})]; %#ok<AGROW>
    end
end
files = files(isfile(files));
stamp = zeros(1, numel(files));
for i = 1:numel(files)
    info = dir(files{i});
    stamp(i) = info.datenum;
end
end

function tf = isCamClipperMethod(method)
% method = planner name, optionally "_<baseId>"; the camClipper family is
% whatever hairsim.planning.camClipperTool knows
plannerName = extractBefore(string(method) + "_", "_");
[~, tf] = hairsim.planning.camClipperTool(plannerName);
end
