function testLaunchers()
%TESTLAUNCHERS The root launchers open their window with this version's
%   code and close cleanly: runAuto and runTeleop are started from outside
%   the app folder; a timer waits for the window to finish starting up
%   (status line present, no longer computing Home) and closes it.

appRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
here = pwd;
cleanup = onCleanup(@() cd(here));
cd(tempdir);
for name = ["runAuto", "runTeleop"]
    t = timer('Period', 1, 'ExecutionMode', 'fixedSpacing', 'TimerFcn', @closeWhenReady, 'UserData', "");
    start(t);
    feval(name);
    stop(t);
    status = t.UserData;
    delete(t);
    source = which('hairsim.tool.createClipperTool');
    fprintf('%s: window closed at "%s", code from %s\n', name, status, source);
    assert(strlength(status) > 0, "%s: the window never finished starting", name);
    assert(startsWith(source, appRoot), "%s ran code outside %s", name, appRoot);
end
fprintf('PASS: launchers\n');
end

function closeWhenReady(tm, ~)
% guard every handle: an error here would stop the timer and leave the
% window open forever
try
    figs = findall(groot, 'Type', 'figure');
    figs = figs(startsWith(string(get(figs, 'Name')), "HairSimApp"));
    if isempty(figs)
        return;
    end
    status = findall(figs(1), 'Tag', 'hairsimStatusText');
    if isempty(status) || contains(string(status.String), "computing")
        return;
    end
    tm.UserData = string(status.String);
    close(figs);
catch
end
end
