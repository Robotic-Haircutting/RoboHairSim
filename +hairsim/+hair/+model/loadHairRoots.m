function roots = loadHairRoots(rootsPath)
%LOADHAIRROOTS Load HairSim scalp-root data from a JSON file.

arguments
    rootsPath (1, :) char
end

if ~isfile(rootsPath)
    error("hairsim:MissingHairRoots", "Cannot open hair roots file: %s", rootsPath);
end

payload = jsondecode(fileread(rootsPath));
if ~isfield(payload, 'roots')
    roots = struct([]);
    return;
end

roots = payload.roots;
if isempty(roots)
    roots = struct([]);
    return;
end

requiredFields = {'position', 'normal', 'region'};
for fieldIdx = 1:numel(requiredFields)
    if ~isfield(roots, requiredFields{fieldIdx})
        error("hairsim:InvalidHairRoots", ...
            "Hair roots file must include a '%s' field.", requiredFields{fieldIdx});
    end
end
end
