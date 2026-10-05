function validateClipperTrajectoryTable(trajectoryTable)
%VALIDATECLIPPERTRAJECTORYTABLE Validate the shared clipper CSV contract.

requiredColumns = {'time','x','y','z','roll','pitch','yaw','cutFlag'};
actualColumns = trajectoryTable.Properties.VariableNames;
missingColumns = setdiff(requiredColumns, actualColumns, 'stable');
if ~isempty(missingColumns)
    error("hairsim:MissingClipperColumns", ...
        "Missing clipper trajectory columns: %s", strjoin(missingColumns, ", "));
end

if height(trajectoryTable) == 0
    error("hairsim:EmptyClipperTrajectory", "Clipper trajectory is empty.");
end

for columnIdx = 1:numel(requiredColumns)
    columnName = requiredColumns{columnIdx};
    values = trajectoryTable.(columnName);
    if ~isnumeric(values) || any(~isfinite(values))
        error("hairsim:InvalidClipperColumn", ...
            "Clipper trajectory column %s must be numeric and finite.", columnName);
    end
end

if any(trajectoryTable.cutFlag ~= 0 & trajectoryTable.cutFlag ~= 1)
    error("hairsim:InvalidClipperCutFlag", "cutFlag values must be 0 or 1.");
end

if any(diff(trajectoryTable.time) < 0)
    error("hairsim:NonMonotonicClipperTime", ...
        "Clipper trajectory time values must be monotonically nondecreasing.");
end
end
