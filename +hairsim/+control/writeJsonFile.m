function writeJsonFile(filePath, value)
%WRITEJSONFILE Write a struct as compact JSON.

fid = hairsim.control.openFileForWrite(filePath, "JSON file");
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(value));
end
