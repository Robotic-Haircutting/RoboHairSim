function teleopState = setHair(teleopState, haircutState, flowDirs)
%SETHAIR Tell Teleoperation where the hair grows and which way it lies,
%   for the G key (feed against the flow) and the feed-angle readout.
%   flowDirs: (numStrands,3), e.g. the installed comb-lift cut model's
%   flowDirs (hairsim.cam.computeStrandFlowDirs). Call again after the hair
%   is rebuilt.

arguments
    teleopState (1, 1) struct
    haircutState (1, 1) struct
    flowDirs (:, 3) double
end

hairs = haircutState.hairState.hairs;
teleopState.hairRoots = [hairs.X(1, :)', hairs.Y(1, :)', hairs.Z(1, :)'];
teleopState.hairFlow = flowDirs;
end
