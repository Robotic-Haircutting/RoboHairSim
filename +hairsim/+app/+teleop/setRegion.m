function teleopState = setRegion(teleopState, regionName)
%SETREGION Confine Teleoperation to one scalp region ("" = the whole
%   head). A move whose foot would leave the region is refused
%   (REJECTED:OUTSIDE_REGION) unless it brings an outside clipper closer to
%   it, so a clipper parked elsewhere can still be driven in
%   (hairsim.app.teleop.regionMembership). Which strands may be CUT is the
%   cut goal's business (hairsim.app.hair.installClipperCut 'Region').

arguments
    teleopState (1, 1) struct
    regionName (1, 1) string
end

if strlength(regionName) > 0 && ~any(teleopState.strandRegions == regionName)
    error("hairsim:UnknownRegion", "No strands in region %s.", regionName);
end
teleopState.regionName = regionName;
teleopState.hud.inRegion = true;
teleopState.hud.regionDistanceMm = 0;
if all(isfinite(teleopState.surface.footWorldM))     % synced: report where the clipper is
    [teleopState.hud.inRegion, distanceM] = hairsim.app.teleop.regionMembership(teleopState, ...
        teleopState.surface.footWorldM);
    teleopState.hud.regionDistanceMm = 1000 * distanceM;
end
end
