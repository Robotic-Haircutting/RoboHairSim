function setupPath()
%SETUPPATH Put this copy of HairSim, and only this one, on the MATLAB path.
%   Every copy of HairSim (e.g. a release and a development checkout)
%   defines the +hairsim package; with two on the path MATLAB silently
%   uses whichever comes first. This removes every other folder that
%   provides +hairsim (and everything under its root), then adds this
%   copy: its root and the test folders it has. runAuto, runTeleop,
%   runFullDemo and replayBundle call it first, so you only need it to use
%   the hairsim.* functions directly:
%
%   setupPath
%   scene = hairsim.api.loadScene();

here = fileparts(mfilename('fullpath'));
entries = strsplit(path, pathsep);
otherRoots = strings(0, 1);
for k = 1:numel(entries)
    p = entries{k};
    if isfolder(fullfile(p, '+hairsim')) && ~startsWith(p, here)
        otherRoots(end + 1) = string(p); %#ok<AGROW>
    end
end
for r = otherRoots'
    under = entries(startsWith(entries, r));
    rmpath(under{:});
    fprintf('setupPath: removed %s (%d path entries)\n', r, numel(under));
end
% entries whose folder no longer exists (a renamed or deleted version):
% harmless now, but they come alive again if a version is later created
% at that place
stale = entries(~cellfun(@isempty, entries) & ~cellfun(@isfolder, entries));
if ~isempty(stale)
    rmpath(stale{:});
    fprintf('setupPath: removed missing folder %s\n', stale{:});
end

% the current folder outranks the path: a +hairsim there would still win
if isfolder(fullfile(pwd, '+hairsim')) && ~strcmp(pwd, here)
    warning("hairsim:OtherVersionInCurrentFolder", ...
        "The current folder %s has its own +hairsim, which MATLAB uses before the path. cd elsewhere (e.g. cd('%s')).", pwd, here);
end

% the root plus whichever of the test folders this copy has
folders = {here, fullfile(here, 'tests', 'camClipper'), ...
    fullfile(here, 'tests', 'app'), fullfile(here, 'tests', 'release')};
addpath(folders{cellfun(@isfolder, folders)});
clear functions             % drop persistent registries/caches built by another version
rehash path
fprintf('setupPath: using %s\n', here);
end
