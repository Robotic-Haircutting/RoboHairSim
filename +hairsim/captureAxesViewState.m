function viewState = captureAxesViewState(ax)
%CAPTUREAXESVIEWSTATE Capture user-controlled camera and axis limits.

[azimuth, elevation] = view(ax);

viewState = struct();
viewState.azimuth = azimuth;
viewState.elevation = elevation;
viewState.xlim = xlim(ax);
viewState.ylim = ylim(ax);
viewState.zlim = zlim(ax);
viewState.cameraPosition = campos(ax);
viewState.cameraTarget = camtarget(ax);
viewState.cameraUpVector = camup(ax);
viewState.cameraViewAngle = camva(ax);
end
