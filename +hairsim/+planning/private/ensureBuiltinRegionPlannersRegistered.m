function ensureBuiltinRegionPlannersRegistered()
%ENSUREBUILTINREGIONPLANNERSREGISTERED Lazily register the shipped presets.
persistent done
if isequal(done, true)
    return;
end
hairsim.planning.registerBuiltinPresets();
done = true;
end
