function variants = edgeVariants(tool)
%EDGEVARIANTS Ordered clipper placements tried where a centred blade does
%   not fit: centred first, then lateral offsets (blade slid sideways so
%   the target sits under one edge), then rolls (one edge dropped onto the
%   scalp), then both. Simplest first, so callers can stop at the first
%   placement that works.
%   variants: struct array with fields name ("centre"|"offset"|"roll"|
%   "offset+roll"), lateralOffsetM, rollRad.

offsets = tool.edgeLateralOffsetsM(:)';
rolls = tool.edgeRollAnglesRad(:)';
variants = struct('name', "centre", 'lateralOffsetM', 0, 'rollRad', 0);
for o = offsets
    for s = [1, -1]
        variants(end + 1) = struct('name', "offset", 'lateralOffsetM', s * o, 'rollRad', 0); %#ok<AGROW>
    end
end
for r = rolls
    for s = [1, -1]
        variants(end + 1) = struct('name', "roll", 'lateralOffsetM', 0, 'rollRad', s * r); %#ok<AGROW>
    end
end
for o = offsets
    for r = rolls
        for s = [1, -1]
            for t = [1, -1]
                variants(end + 1) = struct('name', "offset+roll", 'lateralOffsetM', s * o, 'rollRad', t * r); %#ok<AGROW>
            end
        end
    end
end
end
