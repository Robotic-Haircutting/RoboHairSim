function writeTextFile(filePath, text)
%WRITETEXTFILE Write text to disk, creating the parent folder if needed.

fid = hairsim.control.openFileForWrite(filePath, "text file");
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, '%s', text);
end
