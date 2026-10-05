function trajectory = writeClipperTrajectory(recorder, outputCsvPath)
%WRITECLIPPERTRAJECTORY Validate and write recorded clipper trajectory CSV.

outputDir = fileparts(outputCsvPath);
if ~isempty(outputDir) && ~isfolder(outputDir)
    mkdir(outputDir);
end

trajectory = hairsim.createClipperTrajectoryTable( ...
    recorder.samplesTime, recorder.samplesPosition, ...
    recorder.samplesEuler, recorder.samplesCutFlag);
hairsim.validateClipperTrajectoryTable(trajectory);
writetable(trajectory, outputCsvPath);
end
