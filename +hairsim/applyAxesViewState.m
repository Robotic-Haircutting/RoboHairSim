function applyAxesViewState(ax, viewState)
%APPLYAXESVIEWSTATE Restore a captured axes view after redrawing content.

xlim(ax, viewState.xlim);
ylim(ax, viewState.ylim);
zlim(ax, viewState.zlim);
view(ax, [viewState.azimuth, viewState.elevation]);
campos(ax, viewState.cameraPosition);
camtarget(ax, viewState.cameraTarget);
camup(ax, viewState.cameraUpVector);
camva(ax, viewState.cameraViewAngle);
end
