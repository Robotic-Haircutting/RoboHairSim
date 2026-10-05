function path = computeFollowCameraPath(bundle, robotAppState, scene, T_ee_tcp, options)
%COMPUTEFOLLOWCAMERAPATH A camera that follows the part of the head being
%   cut, for hairsim.app.replayFullDemoBundle('CameraMode', "follow").
%
%   Forward kinematics only (no hair, no rendering): the TCP of every
%   bundle sample, with the rail station / base applied exactly as the
%   replay applies it. View direction = TCP seen from the head centre.
%
%   Regions come from the bundle's statusLabel: each run of "AUTO" samples
%   is the region named on the left of the next bridge label
%   ("Fold scalp_back->Home", "Unfold scalp_top_right->scalp_right",
%   "Home scalp_right->End"). Each region gets ONE base view (mean
%   direction of its cutting samples); inside a region the camera follows
%   the clipper only by FollowWeight; bridge samples (fold, rail slide,
%   unfold) look AHEAD to the next region's view, so the camera turns
%   during the move.
%
%   Arm occlusion: per region, the view is turned by the (azimuth,
%   elevation) offset from AzimuthOffsetsDeg x ElevationAddsDeg that keeps
%   the cut spot visible most often -- the line of sight camera -> TCP is
%   "blocked" when a point sampled along the robot's links (OcclusionPointsPerLink
%   per pair of consecutive body origins) passes within OcclusionRadiusM
%   of it (the last
%   OcclusionIgnoreNearTcpM before the TCP, i.e. the clipper itself, is
%   ignored); checked on every OcclusionStride-th cutting sample. Ties go
%   to the smaller offset. Elevation is clamped to ElevationRangeDeg; the
%   path is finally Gaussian-smoothed over SmoothSigmaSamples (zero phase).
%   path.regionView reports the choice per region.
%
%   path fields (one row / entry per bundle sample): cameraPosition (N,3),
%   cameraTarget (1,3), viewAngleDeg, azimuthDeg (N,1), elevationDeg (N,1),
%   tcpWorldM (N,3), region (N,1 string), regionOrder (string, in order
%   of first appearance).

arguments
    bundle (1, 1) struct
    robotAppState (1, 1) struct
    scene (1, 1) struct
    T_ee_tcp (4, 4) double
    options.SmoothSigmaSamples (1, 1) double {mustBePositive} = 60
    options.FollowWeight (1, 1) double {mustBeInRange(options.FollowWeight, 0, 1)} = 0.3
    options.AzimuthOffsetsDeg (1, :) double = -75:15:75
    options.ElevationAddsDeg (1, :) double = [-30, -15, 0, 15, 30]
    options.OcclusionRadiusM (1, 1) double {mustBePositive} = 0.06
    options.OcclusionPointsPerLink (1, 1) double {mustBePositive, mustBeInteger} = 12
    options.OcclusionIgnoreNearTcpM (1, 1) double {mustBeNonnegative} = 0.09
    options.OcclusionStride (1, 1) double {mustBePositive, mustBeInteger} = 5
    options.ElevationRangeDeg (1, 2) double = [10, 70]
    options.DistanceM (1, 1) double {mustBePositive} = 1.0
    options.ViewAngleDeg (1, 1) double {mustBePositive} = 24
end

numSamples = size(bundle.qCandidate, 2);
centre = scene.headScene.headCenterWorldM(:)';
tcp = nan(numSamples, 3);
cutFlag = logical(bundle.cutFlag(:));
armPoints = cell(numSamples, 1);
applied = nan;
for k = 1:numSamples
    theta = bundle.railThetaRad(k);
    if ~isnan(theta)
        if isnan(applied) || abs(theta - applied) > 1e-9
            robotAppState = hairsim.app.robot.applyRailStationForReplay(robotAppState, theta);
            applied = theta;
        end
    elseif bundle.basePlacementId(k) ~= robotAppState.state.basePlacementId
        robotAppState = hairsim.app.robot.switchBase(robotAppState, bundle.basePlacementId(k));
        applied = nan;
    end
    ctx = robotAppState.modelContext;
    q = bundle.qCandidate(:, k);
    T = getTransform(ctx.robot, q, char(ctx.endEffector)) * T_ee_tcp;
    tcp(k, :) = T(1:3, 4)';
    if cutFlag(k) && mod(k, options.OcclusionStride) == 0
        names = ctx.robot.BodyNames;
        origins = zeros(numel(names), 3);
        for b = 1:numel(names)
            Tb = getTransform(ctx.robot, q, names{b});
            origins(b, :) = Tb(1:3, 4)';
        end
        t = linspace(0, 1, options.OcclusionPointsPerLink)';
        segs = cell(size(origins, 1) - 1, 1);
        for b = 1:size(origins, 1) - 1
            segs{b} = origins(b, :) + t .* (origins(b + 1, :) - origins(b, :));
        end
        armPoints{k} = vertcat(segs{:});
    end
end

% ---- regions from the bridge labels
labels = string(bundle.statusLabel(:));
prefix = extractBefore(labels + ":", ":");
isAuto = prefix == "AUTO";
cut = logical(bundle.cutFlag(:));
region = strings(numSamples, 1);
pending = "";
for k = numSamples:-1:1
    if ~isAuto(k)
        src = regexp(prefix(k), '(scalp_\w+)->', 'tokens', 'once');
        if ~isempty(src)
            pending = string(src{1});
        end
    else
        region(k) = pending;
    end
end
regionOrder = unique(region(region ~= ""), 'stable');

% ---- per region: base view, follow, best occlusion-free offset
d = tcp - centre;
d = d ./ vecnorm(d, 2, 2);
dirs = nan(numSamples, 3);
regionView = struct('region', {}, 'azimuthOffsetDeg', {}, 'elevationAddDeg', {}, 'blockedFraction', {});
for r = regionOrder'
    inRegion = region == r;
    base = mean(d(inRegion & cut, :), 1);
    if ~all(isfinite(base))
        base = mean(d(inRegion, :), 1);
    end
    base = base / norm(base);
    raw = repmat(base, numSamples, 1);
    follow = inRegion & cut;
    mixed = (1 - options.FollowWeight) * base + options.FollowWeight * d(follow, :);
    raw(follow, :) = mixed ./ vecnorm(mixed, 2, 2);
    checkIdx = find(inRegion & ~cellfun(@isempty, armPoints));
    best = struct('blocked', inf, 'cost', inf, 'az', 0, 'el', 0);
    for el = options.ElevationAddsDeg
        for az = options.AzimuthOffsetsDeg
            camDirs = offsetDirs(raw(checkIdx, :), az, el, options.ElevationRangeDeg);
            blocked = 0;
            for j = 1:numel(checkIdx)
                k = checkIdx(j);
                blocked = blocked + isBlocked(centre + options.DistanceM * camDirs(j, :), tcp(k, :), armPoints{k}, ...
                    options.OcclusionRadiusM, options.OcclusionIgnoreNearTcpM);
            end
            frac = blocked / max(1, numel(checkIdx));
            cost = abs(az) + el;
            if frac < best.blocked - 1e-9 || (abs(frac - best.blocked) <= 1e-9 && cost < best.cost)
                best = struct('blocked', frac, 'cost', cost, 'az', az, 'el', el);
            end
        end
    end
    dirs(inRegion, :) = offsetDirs(raw(inRegion, :), best.az, best.el, options.ElevationRangeDeg);
    regionView(end + 1) = struct('region', r, 'azimuthOffsetDeg', best.az, ...
        'elevationAddDeg', best.el, 'blockedFraction', best.blocked); %#ok<AGROW>
end
next = nan(1, 3);
for k = numSamples:-1:1
    if region(k) ~= ""
        next = dirs(k, :);
    elseif all(isfinite(next))
        dirs(k, :) = next;
    end
end
lastIdx = find(region ~= "", 1, 'last');
if isempty(lastIdx)
    dirs(:, :) = repmat([1, 0, 0], numSamples, 1);
else
    dirs(lastIdx + 1:end, :) = repmat(dirs(lastIdx, :), numSamples - lastIdx, 1);
end
halfWidth = ceil(3 * options.SmoothSigmaSamples);
kernel = exp(-0.5 * ((-halfWidth:halfWidth)' / options.SmoothSigmaSamples) .^ 2);
kernel = kernel / sum(kernel);
padded = [repmat(dirs(1, :), halfWidth, 1); dirs; repmat(dirs(end, :), halfWidth, 1)];
cameraDir = conv2(padded, kernel, 'valid');
cameraDir = cameraDir ./ max(vecnorm(cameraDir, 2, 2), eps);
azimuth = unwrap(deg2rad(atan2d(cameraDir(:, 1), -cameraDir(:, 2)))) * 180 / pi;
elevation = asind(cameraDir(:, 3));

path = struct();
path.cameraTarget = centre;
path.cameraPosition = centre + options.DistanceM * cameraDir;
path.viewAngleDeg = options.ViewAngleDeg;
path.azimuthDeg = azimuth;
path.elevationDeg = elevation;
path.tcpWorldM = tcp;
path.region = region;
path.regionOrder = regionOrder;
path.regionView = regionView;
end

function out = offsetDirs(dirs, azOffsetDeg, elAddDeg, elRange)
% view(az, el) convention: camera direction from the target =
% [cos(el) sin(az), -cos(el) cos(az), sin(el)]
az = atan2d(dirs(:, 1), -dirs(:, 2)) + azOffsetDeg;
el = min(max(asind(dirs(:, 3)) + elAddDeg, elRange(1)), elRange(2));
out = [cosd(el) .* sind(az), -cosd(el) .* cosd(az), sind(el)];
end

function tf = isBlocked(cameraPos, tcp, points, radiusM, ignoreNearTcpM)
v = tcp - cameraPos;
L = norm(v);
u = v / L;
rel = points - cameraPos;
t = rel * u';
perp = vecnorm(rel - t * u, 2, 2);
tf = any(t > 0 & t < L - ignoreNearTcpM & perp < radiusM);
end
