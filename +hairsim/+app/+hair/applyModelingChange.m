function [hairAppState, needsRedraw] = applyModelingChange(hairAppState, kind, value)
%APPLYMODELINGCHANGE One Hair Modeling panel change, by kind:
%   "rigidity" | "length" | "density" | "thickness" | "color". needsRedraw:
%   the hair geometry or look changed (everything except rigidity, which
%   only affects the dynamics).

switch kind
    case "rigidity"
        hairAppState = hairsim.app.hair.setRigidity(hairAppState, value);
    case "length"
        hairAppState = hairsim.app.hair.setLength(hairAppState, value);
    case "density"
        hairAppState = hairsim.app.hair.setDensity(hairAppState, value);
    case "thickness"
        hairAppState = hairsim.app.hair.setThickness(hairAppState, value);
    case "color"
        hairAppState = hairsim.app.hair.setColor(hairAppState, value);
    otherwise
        error("hairsim:UnknownHairModelingChange", "Unknown hair modeling change '%s'.", kind);
end
needsRedraw = kind ~= "rigidity";
end
