function model = createHeadCollisionModel(headMeshWorld)
%CREATEHEADCOLLISIONMODEL Robot-body-vs-head collision object for checkCollision.
%   The whole-head convex hull, used as a conservative safety envelope for the
%   robot's own links: the arm must stay outside it (separation >= 0, no extra
%   inflation). It deliberately ignores face/ear/neck concavities.
%   The tool-vs-scalp clearance does NOT use this: it uses the exact mesh,
%   hairsim.evaluateClipperSphereMeshClearance. The hair-physics head hull
%   (headScene.headHullWorld) is a separate model and is not used here.
%   model.objects is a cell array accepted by validateClipperRobotCollision and
%   checkCollision (one element: the hull).

arguments
    headMeshWorld (1, 1) struct
end

points = double(headMeshWorld.vertices);
hullFaces = convhulln(points);
hullObject = collisionMesh(points(unique(hullFaces(:)), :));

model = struct('objects', {{hullObject}}, 'numObjects', 1);
end
