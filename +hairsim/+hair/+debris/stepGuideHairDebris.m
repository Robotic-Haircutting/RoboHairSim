function haircutState = stepGuideHairDebris(haircutState, floorZ)
%STEPGUIDEHAIRDEBRIS Advance falling haircut debris and refresh line data.

arguments
    haircutState (1, 1) struct
    floorZ (1, 1) double
end

if isempty(haircutState.debrisList)
    return;
end

haircutState.debrisList = hairsim.hair.debris.updateGuideHairDebris( ...
    haircutState.debrisList, haircutState.params.debrisGravity, floorZ);
haircutState.lineData = hairsim.hair.render.createHaircutLineData( ...
    haircutState.hairState, haircutState.referenceHairNodeCounts, ...
    haircutState.debrisList);
end
