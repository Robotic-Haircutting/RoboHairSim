function fid = openFileForWrite(filePath, fileKindLabel)
%OPENFILEFORWRITE Ensure filePath's parent folder exists and open it for writing.
%   fileKindLabel appears in the error message on failure (e.g. "JSON file").
%   Shared by hairsim.control.writeJsonFile and hairsim.control.writeTextFile.

outputDir = fileparts(filePath);
if ~isempty(outputDir) && ~isfolder(outputDir)
    mkdir(outputDir);
end

fid = fopen(filePath, 'w');
if fid < 0
    error("hairsim:FileOpenFailed", ...
        "Could not open %s for writing: %s", fileKindLabel, filePath);
end
end
