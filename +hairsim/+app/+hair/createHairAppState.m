function appState = createHairAppState(scene)
%CREATEHAIRAPPSTATE Hair/Sim subsystem state: just the live `hairLayer`
%   (already carries creation/physics/style settings as its own fields --
%   hairsim.hair.rebuildHairLayer already reads those back when rebuilding,
%   so there is no separate mirror of "current" rigidity/length/density/
%   color/thickness to keep in sync; hairLayer itself is the single source
%   of truth).
appState = struct('hairLayer', scene.hairLayer);
end
