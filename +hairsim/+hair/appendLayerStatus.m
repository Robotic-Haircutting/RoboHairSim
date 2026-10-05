function poseStatus = appendLayerStatus(poseStatus, hairUpdateInfo)
%APPENDLAYERSTATUS Add hair update status to a robot pose status label.

hairStatus = hairUpdateInfo.statusLabel;
if hairStatus == "ACCEPTED"
    return;
end
if startsWith(hairStatus, "ACCEPTED:")
    poseStatus = poseStatus + extractAfter(hairStatus, "ACCEPTED");
else
    poseStatus = poseStatus + ":" + hairStatus;
end
end
