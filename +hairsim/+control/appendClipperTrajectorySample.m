function recorder = appendClipperTrajectorySample(recorder, T_world_ee, cutFlag)
%APPENDCLIPPERTRAJECTORYSAMPLE Append one mounted clipper pose sample.

eulerXyz = rotm2eul(T_world_ee(1:3, 1:3), 'XYZ');
recorder.samplesTime(end + 1, 1) = toc(recorder.startTime);
recorder.samplesPosition(end + 1, :) = hairsim.getMountedClipperPoint( ...
    T_world_ee, recorder.T_ee_tool);
recorder.samplesEuler(end + 1, :) = eulerXyz;
recorder.samplesCutFlag(end + 1, 1) = double(cutFlag);
end
