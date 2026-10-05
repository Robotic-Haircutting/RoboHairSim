function configureSceneLighting(ax)
%CONFIGURESCENELIGHTING Keep one bounded light source for live 3-D scenes.

sceneLightTag = 'HairSimSceneLight';
allLights = findobj(ax, 'Type', 'Light');
sceneLights = findobj(ax, 'Type', 'Light', 'Tag', sceneLightTag);

if isempty(sceneLights)
    delete(allLights);
    sceneLight = light(ax, ...
        'Style', 'infinite', ...
        'Position', [1.0, -2.0, 3.0], ...
        'Color', [0.72, 0.72, 0.72], ...
        'Tag', sceneLightTag);
else
    sceneLight = sceneLights(1);
    extraLights = setdiff(allLights, sceneLight);
    delete(extraLights);
end

sceneLight.Style = 'infinite';
sceneLight.Position = [1.0, -2.0, 3.0];
sceneLight.Color = [0.72, 0.72, 0.72];
lighting(ax, 'gouraud');
end
