function teleopState = setHair(teleopState, haircutState, flowDirs)
%SETHAIR Tell Teleoperation where the hair grows and which way it lies,
%   for the G key (feed against the flow) and the feed-angle readout.
%   flowDirs: (numStrands,3), e.g. the installed comb-lift cut model's
%   flowDirs (hairsim.cam.computeStrandFlowDirs). Call again after the hair
%   is rebuilt.
%   Also the region labels and reach for the region limit
%   (hairsim.app.teleop.regionMembership): reach = 3x the median spacing
%   between neighbouring roots.

arguments
    teleopState (1, 1) struct
    haircutState (1, 1) struct
    flowDirs (:, 3) double
end

hairs = haircutState.hairState.hairs;
roots = [hairs.X(1, :)', hairs.Y(1, :)', hairs.Z(1, :)'];
teleopState.hairRoots = roots;
teleopState.hairFlow = flowDirs;
teleopState.strandRegions = hairsim.hair.getStrandRegions(haircutState);
if strlength(teleopState.regionName) > 0 && ~any(teleopState.strandRegions == teleopState.regionName)
    teleopState.regionName = "";
end
spacing = zeros(size(roots, 1), 1);
rootSq = sum(roots .^ 2, 2)';
for first = 1:2000:size(roots, 1)
    rows = first:min(first + 1999, size(roots, 1));
    D2 = max(0, sum(roots(rows, :) .^ 2, 2) + rootSq - 2 * (roots(rows, :) * roots'));
    D2(sub2ind(size(D2), 1:numel(rows), rows)) = Inf;
    spacing(rows) = sqrt(min(D2, [], 2));
end
teleopState.regionReachM = 3 * median(spacing);
end
