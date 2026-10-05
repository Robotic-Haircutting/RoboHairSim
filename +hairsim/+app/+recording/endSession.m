function recordingState = endSession(recordingState)
%ENDSESSION Mark recording inactive. Does not finalize (write CSVs/
%   metrics) -- that is hairsim.app.recording.finalizeSession, called
%   separately once the caller has appended the SESSION_END sample.
recordingState.isActive = false;
end
