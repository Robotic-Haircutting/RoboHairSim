function [regionPlan, post] = postProcessToolpaths(toolpathPlan, baseCtx, headQuery, haircutState, options)
%POSTPROCESSTOOLPATHS The robot layer of the camClipper pipeline (CNC
%   "post-processor"): tool-position file -> joint trajectories.
%
%   For each CAM segment, in order:
%     1. try rail stations (the last good one first, then options.StationsDeg
%        in list order, or nearest first along the ring with StationOrder
%        "nearest") until one walks the whole segment
%        (hairsim.post.walkClipperSegment); if none can and RefineStepDeg is
%        set, also try the angles within RefineHalfWidthDeg of the station
%        that got furthest (the rail can stop anywhere, the list is only a
%        search grid);
%     2. if none can, split the segment's CUT samples in two (overlapping by
%        one sample, each half with its own approach/retreat) and retry each
%        half, down to options.MaxSplitDepth -- "only partly reachable: run
%        the reachable parts";
%     3. a piece nobody can walk is recorded with the dominant rejection of
%        the station that got furthest.
%   Executable pieces become strokes; consecutive strokes are connected by
%   the Hybrid transition (hairsim.planning.planSafeConfigurationTransition),
%   through Home with an instantaneous rail switch when the stations differ,
%   and every transit sample is re-checked with the clipper model. A stroke
%   no transit can reach is left out of the chain.
%
%   regionPlan follows the planner contract every consumer reads (GUI
%   preview, hairsim.execution.loadPlan, caching, full demo), one stroke per
%   executable piece. post carries the evaluation B inputs and the
%   per-piece outcomes (see hairsim.eval.compareIdealVsRobot).

arguments
    toolpathPlan (1, 1) struct
    baseCtx (1, 1) struct
    headQuery (1, 1) struct
    haircutState (1, 1) struct
    % rail angles are on a full ring: normalised to [0, 360), slides take
    % the shorter way round
    options.StationsDeg (1, :) double = [180, 165, 150, 135, 195, 210, 120, 225, 240]
    options.StationOrder (1, 1) string {mustBeMember(options.StationOrder, ["list", "nearest"])} = "list"
    options.RefineStepDeg (1, 1) double = NaN          % NaN = grid only
    options.RefineHalfWidthDeg (1, 1) double {mustBePositive} = 7.5
    options.MaxSplitDepth (1, 1) double {mustBeNonnegative, mustBeInteger} = 3
    options.MinPieceCutSamples (1, 1) double {mustBePositive, mustBeInteger} = 2
    options.DeadEndPatience (1, 1) double {mustBePositive, mustBeInteger} = 3
    options.InsertSearchDepth (1, 1) double {mustBeNonnegative, mustBeInteger} = 6
    options.PlannerName (1, 1) string = "camClipperAgainstFlow"
    options.OnProgress = []
end

timer = tic;
tool = toolpathPlan.tool;
if ~isfield(baseCtx, 'railContextBuilder')
    options.StationsDeg = NaN;          % fixed base: one station, the context's own
end
% clipper-safe park pose (the GUI Home leaves the real blade inside the
% head); transits through "Home" and rail switches use it, and stations
% where it is not clear are not used
parkReport = [];
if isnan(options.StationsDeg(1))
    % fixed base (incl. ceiling): its own clipper-safe park pose, adapter Home as fallback
    try
        [qPark, parkReport] = hairsim.post.findClipperParkConfiguration(baseCtx, tool, headQuery, NaN);
    catch parkError
        % the adapter Home may leave the
        % real clipper at / inside the head: say so instead of hiding it
        warning("hairsim:post:NoClipperParkPose", ...
            "No clipper-safe park pose on this base (%s); transits use the adapter Home.", parkError.message);
        qPark = baseCtx.qHome(:);
    end
else
    options.StationsDeg = unique(mod(options.StationsDeg, 360), 'stable');
    [qPark, parkReport] = hairsim.post.findClipperParkConfiguration(baseCtx, tool, headQuery, options.StationsDeg);
    options.StationsDeg = parkReport.validStationsDeg;
    report(options.OnProgress, sprintf("post %s: park pose %.0f mm off Home, valid at %s deg", ...
        toolpathPlan.region, 1000 * parkReport.liftM, mat2str(options.StationsDeg)));
end
params = createParams();
stations = containers.Map('KeyType', 'double', 'ValueType', 'any');
    function sctx = stationCtx(thetaDeg)
        thetaDeg = normDeg(thetaDeg);
        key = thetaDeg;
        if isnan(key)
            key = -1;
        end
        if ~isKey(stations, key)
            stations(key) = hairsim.post.createStationContext(baseCtx, deg2rad(thetaDeg), tool);
        end
        sctx = stations(key);
    end

% ---------- 1-3: pieces -> strokes
queue = struct('segIdx', num2cell(1:numel(toolpathPlan.segments)), 'cutRange', [], 'depth', 0);
for i = 1:numel(queue)
    queue(i).cutRange = [1, size(toolpathPlan.segments(i).T_cut, 3)];
end
pieces = struct('segIdx', {}, 'cutRange', {}, 'ok', {}, 'thetaDeg', {}, 'walk', {}, ...
    'T_plan', {}, 'phase', {}, 'reason', {}, 'failedPhase', {});
lastTheta = options.StationsDeg(1);
while ~isempty(queue)
    item = queue(1);
    queue(1) = [];
    seg = toolpathPlan.segments(item.segIdx);
    idx = item.cutRange(1):item.cutRange(2);
    [Tplan, phase] = hairsim.cam.withApproachRetreat(seg.T_cut(:, :, idx), ...
        seg.normalWorldM(idx(1), :), seg.normalWorldM(idx(end), :), 0.02, 0.005);
    report(options.OnProgress, sprintf("post %s: segment %d/%d cut %d-%d (depth %d)", ...
        toolpathPlan.region, item.segIdx, numel(toolpathPlan.segments), idx(1), idx(end), item.depth));
    order = stationOrder(lastTheta, options.StationsDeg, options.StationOrder);
    best = [];
    bestProgress = -1;
    accepted = [];
    tried = zeros(1, 0);
    for pass = 1:2
        for th = order
            walk = hairsim.post.walkClipperSegment(Tplan, phase, stationCtx(th), params, tool, headQuery);
            tried(end + 1) = th; %#ok<AGROW>
            if walk.ok
                accepted = walk;
                acceptedTheta = th;
                break;
            end
            if walk.failedSample > bestProgress
                bestProgress = walk.failedSample;
                best = walk;
                bestTheta = th;
            end
        end
        if ~isempty(accepted) || isnan(options.RefineStepDeg) || isnan(lastTheta)
            break;
        end
        % refine around the station that got furthest
        offsets = options.RefineStepDeg:options.RefineStepDeg:options.RefineHalfWidthDeg;
        order = normDeg(bestTheta + reshape([offsets; -offsets], 1, []));
        order = order(~ismember(round(order, 6), round(tried, 6)));
    end
    if ~isempty(accepted)
        lastTheta = acceptedTheta;
        pieces(end + 1) = struct('segIdx', item.segIdx, 'cutRange', item.cutRange, 'ok', true, ...
            'thetaDeg', acceptedTheta, 'walk', accepted, 'T_plan', Tplan, 'phase', phase, ...
            'reason', "OK", 'failedPhase', ""); %#ok<AGROW>
        continue;
    end
    numCut = numel(idx);
    if item.depth < options.MaxSplitDepth && numCut >= 2 * options.MinPieceCutSamples
        mid = item.cutRange(1) + floor(numCut / 2);
        left = struct('segIdx', item.segIdx, 'cutRange', [item.cutRange(1), mid], 'depth', item.depth + 1);
        right = struct('segIdx', item.segIdx, 'cutRange', [mid, item.cutRange(2)], 'depth', item.depth + 1);
        queue = [left, right, queue]; %#ok<AGROW>
        continue;
    end
    pieces(end + 1) = struct('segIdx', item.segIdx, 'cutRange', item.cutRange, 'ok', false, ...
        'thetaDeg', bestTheta, 'walk', best, 'T_plan', Tplan, 'phase', phase, ...
        'reason', best.reason, 'failedPhase', best.failedPhase); %#ok<AGROW>
end

% ---------- strokes + transits (chain)
okPieces = find([pieces.ok]);
[pieces.strokeIdx] = deal(0);
for s = 1:numel(okPieces)
    pieces(okPieces(s)).strokeIdx = s;
end
strokes = repmat(hairsim.planning.emptyStroke(0), 1, numel(okPieces));
allLanes = cell(1, numel(okPieces));
for s = 1:numel(okPieces)
    p = pieces(okPieces(s));
    allLanes{s} = toolpathPlan.segments(p.segIdx).footWorldM(p.cutRange(1):p.cutRange(2), :);
    strokes(s) = makeStroke(s, p, allLanes{s}, baseCtx, tool);
end
transits = repmat(hairsim.planning.emptyTransit(), 1, 0);
chain = zeros(1, 0);
    function ok = tryLink(i, j)
        report(options.OnProgress, sprintf("post %s: transit %d->%d", toolpathPlan.region, i, j));
        transit = planTransit(strokes(i), strokes(j), @stationCtx, qPark, tool, headQuery, numel(options.StationsDeg) > 1);
        transit.fromStroke = i;
        transit.toStroke = j;
        transits(end + 1) = transit;
        ok = transit.feasible;
    end
if ~isempty(strokes)
    % Greedy in CAM order. A stroke the chain cannot reach is deferred and
    % retried from the chain's end after the pass; a chain end that reaches
    % none of the next options.DeadEndPatience strokes is a dead end and is
    % backed off (deferred too) so it cannot strand the rest of the region.
    chain = 1;
    deferred = zeros(1, 0);
    failsFromEnd = 0;
    j = 2;
    while j <= numel(strokes)
        if tryLink(chain(end), j)
            chain(end + 1) = j; %#ok<AGROW>
            failsFromEnd = 0;
            j = j + 1;
            continue;
        end
        failsFromEnd = failsFromEnd + 1;
        if failsFromEnd >= options.DeadEndPatience
            deferred(end + 1) = chain(end); %#ok<AGROW>
            failsFromEnd = 0;
            if numel(chain) > 1
                chain(end) = [];
                continue;                    % retry j from the new chain end
            end
            chain = j;                       % the first stroke was the dead end: restart at j
            j = j + 1;
            continue;
        end
        deferred(end + 1) = j; %#ok<AGROW>
        j = j + 1;
    end
    progress = true;
    while progress && ~isempty(deferred)
        progress = false;
        for d = deferred
            if any(chain == d)
                continue;
            end
            if tryLink(chain(end), d)
                chain(end + 1) = d; %#ok<AGROW>
                progress = true;
            end
        end
        deferred = setdiff(deferred, chain, 'stable');
    end
    % The chain is entered from and left to the park pose (region to region,
    % rail moves): its first stroke must be reachable from it and its last
    % stroke must be able to get back to it. An end stroke that cannot is
    % dropped (a deferred stroke appended last can be such a dead end).
    while ~isempty(chain)
        last = strokes(chain(end));
        [~, okExit] = runLegs({{last.planResult.replayPositions(:, end), qPark, rad2deg(last.railThetaRad)}}, ...
            @stationCtx, tool, headQuery);
        if okExit
            break;
        end
        report(options.OnProgress, sprintf("post %s: stroke %d cannot return to park, dropped from the chain end", ...
            toolpathPlan.region, chain(end)));
        chain(end) = [];
    end
    while ~isempty(chain)
        first = strokes(chain(1));
        [~, okEntry] = runLegs({{qPark, first.planResult.replayPositions(:, 1), rad2deg(first.railThetaRad)}}, ...
            @stationCtx, tool, headQuery);
        if okEntry
            break;
        end
        report(options.OnProgress, sprintf("post %s: stroke %d not reachable from park, dropped from the chain start", ...
            toolpathPlan.region, chain(1)));
        chain(1) = [];
    end
    % Recovery: an executable stroke left out of the chain (no transit in,
    % or a dead end that cannot get out) is re-walked at the OTHER rail
    % stations -- the arm configuration at its ends, and so its way in and
    % out, changes with the base. The first variant that links from the
    % chain end and can still return to the park pose is appended.
    if ~isempty(chain)
        for s0 = setdiff(1:numel(okPieces), chain, 'stable')
            p = pieces(okPieces(s0));
            others = stationOrder(p.thetaDeg, options.StationsDeg, options.StationOrder);
            for th = others(2:end)
                walk = hairsim.post.walkClipperSegment(p.T_plan, p.phase, stationCtx(th), params, tool, headQuery);
                if ~walk.ok
                    continue;
                end
                p.walk = walk;
                p.thetaDeg = th;
                cand = numel(strokes) + 1;
                allLanes{cand} = allLanes{s0};
                strokes(cand) = makeStroke(cand, p, allLanes{cand}, baseCtx, tool);
                linked = tryLink(chain(end), cand);
                okExit = false;
                if linked
                    [~, okExit] = runLegs({{strokes(cand).planResult.replayPositions(:, end), qPark, th}}, ...
                        @stationCtx, tool, headQuery);
                end
                if linked && okExit
                    chain(end + 1) = cand; %#ok<AGROW>
                    pieces(okPieces(s0)).strokeIdx = cand;
                    report(options.OnProgress, sprintf("post %s: stroke %d recovered at %g deg", ...
                        toolpathPlan.region, s0, th));
                    break;
                end
                if linked
                    transits(end).feasible = false;      % never reuse a link into a discarded candidate
                    transits(end).reason = "DISCARDED_CANDIDATE_NO_EXIT";
                end
                strokes(cand) = [];
                allLanes(cand) = [];
            end
        end
        % Recovery 2: a stroke still missing (e.g. one that cannot be left
        % at any station on its own) is inserted BETWEEN two chained
        % strokes, near the chain end first: the transit out of it may then
        % use a rail slide exit (planTransit) on its way to the next stroke.
        for s0 = 1:numel(okPieces)
            if ismember(pieces(okPieces(s0)).strokeIdx, chain)
                continue;
            end
            for k = numel(chain) - 1:-1:max(1, numel(chain) - options.InsertSearchDepth)
                if ~tryLink(chain(k), s0)
                    continue;
                end
                linkIn = numel(transits);
                if ~tryLink(s0, chain(k + 1))
                    % the way in is useless without a way out: do not leave
                    % it counted as a feasible transit
                    transits(linkIn).feasible = false;
                    transits(linkIn).reason = "DISCARDED_INSERT_NO_EXIT";
                    continue;
                end
                chain = [chain(1:k), s0, chain(k + 1:end)];
                pieces(okPieces(s0)).strokeIdx = s0;
                report(options.OnProgress, sprintf("post %s: stroke %d inserted after stroke %d", ...
                    toolpathPlan.region, s0, chain(k)));
                break;
            end
        end
    end
end

% ---------- evaluation B inputs
surface = toolpathPlan.surface;
needs = surface.needsCut;
coveredExec = false(size(needs));
coveredChain = false(size(needs));
for s = 1:numel(strokes)
    T_cut_actual = strokes(s).primitive.T_actual_tcp(:, :, strokes(s).samplePhase == "CUT");
    hit = hairsim.eval.sweepCombLiftCoverage(surface.rootsWorldM, T_cut_actual, tool, ...
        'FlowDirs', surface.flowDirs);
    coveredExec = coveredExec | hit;
    if any(chain == s)
        coveredChain = coveredChain | hit;
    end
end

regionPlan = assembleRegionPlan(toolpathPlan, haircutState, strokes, transits, chain, allLanes, ...
    coveredExec, coveredChain, params, baseCtx, options.PlannerName);
post = struct('pieces', pieces, 'chain', chain, 'coveredExec', coveredExec, 'coveredChain', coveredChain, ...
    'C_exec', mean(coveredExec(needs)), 'C_B', mean(coveredChain(needs)), ...
    'stationsUsedDeg', unique([pieces([pieces.ok]).thetaDeg]), 'qPark', qPark, ...
    'parkReport', parkReport, 'planningTimeS', toc(timer));
regionPlan.planningTimeS = post.planningTimeS;
report(options.OnProgress, sprintf("post %s: %d/%d pieces executable, chain %d, C_B %.1f%% (%.0f s)", ...
    toolpathPlan.region, numel(okPieces), numel(pieces), numel(chain), 100 * post.C_B, post.planningTimeS));
end

% =========================================================================

function params = createParams()
params = struct();
params.ikWeights = [0.01, 0.01, 0.01, 5, 5, 5];
params.maxToolTipErrorM = 0.002;
params.collisionMarginM = 0.001;
params.maxJointStepRad = deg2rad(60);
end

function stroke = makeStroke(strokeIdx, piece, lane, baseCtx, tool)
walk = piece.walk;
stroke = hairsim.planning.emptyStroke(strokeIdx);
stroke.executable = true;
stroke.reason = "OK";
stroke.railThetaRad = deg2rad(piece.thetaDeg);
stroke.scalpCutPathWorldM = lane;
cutMask = piece.phase == "CUT";
plannedTcp = squeeze(piece.T_plan(1:3, 4, :))';
actualTcp = squeeze(walk.T_actual(1:3, 4, :))';
stroke.safeCutPathWorldM = plannedTcp(cutMask, :);
stroke.achievedPathWorldM = actualTcp;
stroke.achievedCutPathWorldM = actualTcp(cutMask, :);
stroke.achievedCutSegmentsWorldM = {actualTcp(cutMask, :)};
stroke.samplePhase = piece.phase;
stroke.primitive = struct('T_plan_tcp', piece.T_plan, 'T_actual_tcp', walk.T_actual, ...
    'samplePhase', piece.phase, 'segIdx', piece.segIdx, 'cutRange', piece.cutRange);
% replay targets in the toolMount tip convention, so the execution status
% label's tip error compares like with like
numSamples = size(piece.T_plan, 3);
tipTargets = zeros(numSamples, 3);
for k = 1:numSamples
    T_ee = piece.T_plan(:, :, k) / tool.T_ee_tcp;
    tipTargets(k, :) = hairsim.getMountedClipperPoint(T_ee, baseCtx.toolMount.T_ee_tool);
end
stroke.planResult = struct( ...
    'strokePlan', struct('plan', struct('status', "ok"), 'replayTipTargetsWorldM', tipTargets), ...
    'summary', struct('success', true, 'failureReason', ""), ...
    'replayPositions', walk.q, 'cutMask', cutMask', 'worldObstacleCount', 1, ...
    'replayable', true, 'message', "OK");
end

function transit = planTransit(a, b, stationCtx, qPark, tool, headQuery, allowRailSlide)
% Stroke a's last sample -> stroke b's first sample. Candidate routes, in
% order (first feasible wins; every leg and every rail step re-checked
% with the exact clipper model):
%   same station:  direct; through the park pose
%   other station: to park, slide the rail (q held at park), to b
%   rail "slide exit" (only with more than one station, i.e. a real rail;
%   a single station is a fixed base): hold a's last pose
%   and slide the rail by +-5..30 deg, leave to park there, slide on to
%   b's station, go to b.
% Rail slides are animated: one sample per RailStepDeg with q held, each
% with its own station switch (hairsim.execution serves them in order).
transit = hairsim.planning.emptyTransit();
timer = tic;
qa = a.planResult.replayPositions(:, end);
qb = b.planResult.replayPositions(:, 1);
thA = rad2deg(a.railThetaRad);
thB = rad2deg(b.railThetaRad);
routes = {};
if (isnan(thA) && isnan(thB)) || abs(ringDelta(thA, thB)) < 1e-9
    routes{end + 1} = {{'leg', qa, qb, thA}};
    routes{end + 1} = {{'leg', qa, qPark, thA}, {'leg', qPark, qb, thA}};
else
    routes{end + 1} = {{'leg', qa, qPark, thA}, {'slide', qPark, thA, thB}, {'leg', qPark, qb, thB}};
end
if allowRailSlide && isfinite(thA)
    for dth = [5, -5, 10, -10, 15, -15, 20, -20, 30, -30]
        thX = normDeg(thA + dth);
        routes{end + 1} = {{'slide', qa, thA, thX}, {'leg', qa, qPark, thX}, ...
            {'slide', qPark, thX, thB}, {'leg', qPark, qb, thB}}; %#ok<AGROW>
    end
end
reason = "NO_ROUTE";
for r = 1:numel(routes)
    [Q, switchIdx, switchTheta, ok, reason] = runMoves(routes{r}, stationCtx, tool, headQuery, thA);
    if ok
        break;
    end
end
transit.feasible = ok;
transit.reason = reason;
transit.planningTimeS = toc(timer);
if ~ok
    return;
end
transit.samplePhase = repmat("TRANSIT", size(Q, 2), 1);
transit.achievedPathWorldM = fkPath(Q, thA, switchIdx, switchTheta, stationCtx, tool);
transit.planResult = struct('strokePlan', struct('plan', struct('status', "ok")), ...
    'summary', struct('success', true, 'failureReason', ""), 'replayPositions', Q, ...
    'cutMask', false(1, size(Q, 2)), 'worldObstacleCount', 1, 'replayable', true, 'message', "OK");
if ~isempty(switchIdx)
    transit.usedRailSlide = true;
    transit.railSlideThetaRad = deg2rad(switchTheta);
    transit.stationSwitchSampleIdx = switchIdx;
end
end

function [Q, switchIdx, switchTheta, ok, reason] = runMoves(moves, stationCtx, tool, headQuery, ~)
% 'leg': hairsim.planning.planSafeConfigurationTransition at one station
% (runLegs); 'slide': rail steps of railStepDeg with q held, each step
% checked (clipper vs exact head, robot links vs head + rail environment,
% arm + clipper self-collision) and recorded as its own sample + switch.
railStepDeg = 2;
Q = zeros(numel(moves{1}{2}), 0);
switchIdx = zeros(1, 0);
switchTheta = zeros(1, 0);
ok = true;
reason = "OK";
for m = 1:numel(moves)
    mv = moves{m};
    if strcmp(mv{1}, 'leg')
        [Qleg, ok, reason] = runLegs({{mv{2}, mv{3}, mv{4}}}, stationCtx, tool, headQuery);
        if ~ok
            return;
        end
        Q = [Q, Qleg]; %#ok<AGROW>
        continue;
    end
    [q, th0, th1] = mv{2:4};
    if th0 == th1
        continue;
    end
    delta = ringDelta(th0, th1);       % shorter way round the ring
    if abs(delta) < 1e-9
        continue;
    end
    steps = th0 + sign(delta) * (railStepDeg:railStepDeg:abs(delta));
    if isempty(steps) || abs(steps(end) - (th0 + delta)) > 1e-9
        steps(end + 1) = th0 + delta; %#ok<AGROW>
    end
    steps = normDeg(steps);
    for th = steps
        sctx = stationCtx(th);
        T = getTransform(sctx.robot, q, char(sctx.endEffector)) * tool.T_ee_tcp;
        robotClear = hairsim.motion.validateClipperRobotCollision(sctx.robot, q, sctx.endEffector, ...
            sctx.headCollision, 0.001, true);
        if ~hairsim.tool.evaluateClipperClearance(T, tool, headQuery).ok || ~robotClear || ...
                checkCollision(sctx.armWithToolRobot, q, 'SkippedSelfCollisions', 'parent')
            ok = false;
            reason = sprintf("RAIL_SLIDE_BLOCKED_AT_%gDEG", th);
            return;
        end
        Q = [Q, q]; %#ok<AGROW>
        switchIdx(end + 1) = size(Q, 2); %#ok<AGROW>
        switchTheta(end + 1) = th; %#ok<AGROW>
    end
end
if isempty(Q)
    Q = moves{1}{2};                 % degenerate: nothing to move
end
end

function [Q, ok, reason, legLengths] = runLegs(legs, stationCtx, tool, headQuery)
% Each leg: hairsim.planning.planSafeConfigurationTransition at one station,
% with the clipper model and the exact head mesh (the same gates as a cut).
Q = zeros(numel(legs{1}{1}), 0);
legLengths = zeros(1, numel(legs));
ok = true;
reason = "OK";
for l = 1:numel(legs)
    [qa, qb, th] = legs{l}{:};
    sctx = stationCtx(th);
    sctx.tool = tool;
    sctx.headQuery = headQuery;
    if max(abs(qa - qb)) < 1e-9
        legLengths(l) = 0;
        continue;
    end
    res = hairsim.planning.planSafeConfigurationTransition(qa, qb, sctx);
    if ~res.feasible
        ok = false;
        reason = "TRANSIT_" + string(res.reason);
        return;
    end
    Q = [Q, res.Q]; %#ok<AGROW>
    legLengths(l) = size(res.Q, 2);
end
end

function path = fkPath(Q, thStart, switchIdx, switchTheta, stationCtx, tool)
path = zeros(size(Q, 2), 3);
th = thStart;
for k = 1:size(Q, 2)
    hit = find(switchIdx == k, 1, 'last');
    if ~isempty(hit)
        th = switchTheta(hit);
    end
    sctx = stationCtx(th);
    T = getTransform(sctx.robot, Q(:, k), char(sctx.endEffector)) * tool.T_ee_tcp;
    path(k, :) = T(1:3, 4)';
end
end

function regionPlan = assembleRegionPlan(tp, haircutState, strokes, transits, chain, allLanes, ...
        coveredExec, coveredChain, params, baseCtx, plannerName)
surface = tp.surface;
needs = surface.needsCut;
regionPlan = struct();
regionPlan.region = tp.region;
regionPlan.plannerName = plannerName;
regionPlan.targetLengthM = median(haircutState.lengthGoal.targetRemainingLengthM(surface.strandIdx));
regionPlan.lengthGoal = haircutState.lengthGoal;
regionPlan.hairActiveNodeCounts = haircutState.hairState.activeNodeCounts;
regionPlan.numTargetsRegion = nnz(needs);
regionPlan.numTargetsTotal = nnz(needs);
regionPlan.numLanes = numel(allLanes);
regionPlan.allLanesWorldM = allLanes;
regionPlan.targetCutNodesWorldM = surface.rootsWorldM(needs, :);
regionPlan.strokes = strokes;
regionPlan.numStrokes = numel(strokes);
regionPlan.numExecutable = numel(strokes);
regionPlan.executable = ~isempty(strokes);
regionPlan.transitsPlanned = true;
regionPlan.transits = transits;
regionPlan.chainStrokeIdx = chain;
regionPlan.numTransitsTried = numel(transits);
regionPlan.numTransitsFeasible = nnz([transits.feasible]);
regionPlan.numRailSlidesUsed = nnz([transits.usedRailSlide]);
regionPlan.coverage = struct('scalp', tp.C_A, 'safe', tp.C_A, ...
    'exec', mean(coveredExec(needs)), 'chain', mean(coveredChain(needs)));
regionPlan.targetCoveredSafe = tp.covered(needs);
regionPlan.targetCoveredExec = coveredExec(needs);
regionPlan.targetCovered = coveredChain(needs);
regionPlan.params = params;
regionPlan.qStartAtPlan = baseCtx.qHome;
regionPlan.runStrokeIdx = nan;
regionPlan.laneIdx = nan;
regionPlan.planResult = [];
if regionPlan.executable
    regionPlan.runStrokeIdx = chain(1);
    regionPlan.laneIdx = chain(1);
    regionPlan.planResult = strokes(chain(1)).planResult;
    regionPlan.message = sprintf("%d strokes executable, %d in the chain.", numel(strokes), numel(chain));
else
    regionPlan.message = "No executable stroke.";
end
end

function report(fn, msg)
if ~isempty(fn)
    fn(char(msg));
end
end

function th = normDeg(th)
% rail angle on the ring, in [0, 360); NaN (fixed base) stays NaN
th = mod(round(th, 9), 360);
end

function d = ringDelta(th0, th1)
% signed shortest rotation from th0 to th1 (deg), in (-180, 180]
d = mod(th1 - th0 + 180, 360) - 180;
if d == -180
    d = 180;
end
end

function order = stationOrder(thLast, stationsDeg, mode)
% thLast first, then the other stations in list order ("list") or by ring
% distance from thLast ("nearest")
if isnan(thLast)
    order = NaN;
    return;
end
others = stationsDeg(abs(arrayfun(@(t) ringDelta(thLast, t), stationsDeg)) > 1e-9);
if mode == "nearest"
    [~, k] = sort(abs(arrayfun(@(t) ringDelta(thLast, t), others)));
    others = others(k);
end
order = [thLast, others];
end
