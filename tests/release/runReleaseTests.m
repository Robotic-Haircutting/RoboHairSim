function results = runReleaseTests(options)
%RUNRELEASETESTS The test suite a release build must pass (run it from the
%   app root after setupPath, or let it call setupPath itself):
%     foundations  clipper model, exact head distance, comb-lift coverage
%                  predicate and cut (testCamClipperFoundations)        ~10 s
%     teleop       headless clipper Teleop on the scalp: safety, cut ==
%                  predicate, no cut pulling back / with the flow
%                  (testTeleopClipper)                                  ~1 min
%     teleopRegion Teleop confined to one region: highlight, moves stop at
%                  the boundary, only its strands cut (testTeleopRegion) ~1 min
%     teleopRail   Teleop rail slide (C / V): one step per press, clipper
%                  held, ring limit, no rail on fixed (testTeleopRail)  ~1 min
%     launchers    runAuto / runTeleop open and close (testLaunchers)   ~1 min
%     autoRegion   plan scalp_back and execute it headless: executed cut ==
%                  the planner's evaluation B (testAutoRegion)          ~10 min
%   options.Quick skips autoRegion. Errors if any test fails; results is a
%   table (name, passed, seconds, message).

arguments
    options.Quick (1, 1) logical = false
end

appRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
run(fullfile(appRoot, 'setupPath.m'));

tests = {
    "foundations", @() testCamClipperFoundations()
    "teleop", @() testTeleopClipper()
    "teleopRegion", @() testTeleopRegion()
    "teleopRail", @() testTeleopRail()
    "launchers", @() testLaunchers()
    "autoRegion", @() testAutoRegion()};
if options.Quick
    tests = tests(1:5, :);
end
numTests = size(tests, 1);
results = table('Size', [numTests, 4], 'VariableTypes', ["string", "logical", "double", "string"], ...
    'VariableNames', ["name", "passed", "seconds", "message"]);
for k = 1:numTests
    fprintf('\n===== %s =====\n', tests{k, 1});
    started = tic;
    results.name(k) = tests{k, 1};
    try
        tests{k, 2}();
        results.passed(k) = true;
        results.message(k) = "";
    catch testError
        results.passed(k) = false;
        results.message(k) = string(testError.message);
        fprintf(2, 'FAILED: %s\n', testError.message);
    end
    results.seconds(k) = toc(started);
end
fprintf('\n');
disp(results);
if ~all(results.passed)
    error("hairsim:ReleaseTestsFailed", "%d of %d release tests failed.", nnz(~results.passed), numTests);
end
fprintf('ALL %d RELEASE TESTS PASSED\n', numTests);
end
