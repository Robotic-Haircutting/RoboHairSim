function debrisList = updateGuideHairDebris(debrisList, gravity, floorZ, options)
%UPDATEGUIDEHAIRDEBRIS Advance falling guide-hair debris by one visual step.
%   Debris that has come to rest on the floor is kept, visible, for
%   SettledLifetimeSteps more calls (so a cut is still seen landing), then
%   dropped from the list -- otherwise every piece of hair ever cut would
%   stay drawn for the rest of the session.

arguments
    debrisList (1, :) struct
    gravity (1, 3) double
    floorZ (1, 1) double
    options.SettledLifetimeSteps (1, 1) double {mustBeNonnegative} = 60
end

keep = true(1, numel(debrisList));
for debrisIdx = 1:numel(debrisList)
    debris = debrisList(debrisIdx);
    falling = ~debris.isStopped;
    debris.vz(falling) = debris.vz(falling) + gravity(3) * 1.5;
    debris.X(falling) = debris.X(falling) + debris.vx(falling);
    debris.Y(falling) = debris.Y(falling) + debris.vy(falling);
    debris.Z(falling) = debris.Z(falling) + debris.vz(falling);

    hitFloor = debris.Z <= floorZ;
    if any(hitFloor)
        debris.Z(hitFloor) = floorZ;
        debris.vz(hitFloor) = 0;
        debris.vx(hitFloor) = 0;
        debris.vy(hitFloor) = 0;
        debris.isStopped(hitFloor) = true;
    end

    if all(debris.isStopped)
        debris.stopAge = debris.stopAge + 1;
    else
        debris.stopAge = 0;
    end
    if debris.stopAge > options.SettledLifetimeSteps
        keep(debrisIdx) = false;
    end
    debrisList(debrisIdx) = debris;
end
debrisList = debrisList(keep);
end
