function haircutState = stepHaircutPhysics(haircutState, headCollision, options)
%STEPHAIRCUTPHYSICS Advance reusable haircut hair dynamics.
%   headCollision: the exact head mesh query
%   (hairsim.tool.createHeadDistanceQuery), or [] for none.
%   After the constraints and the collision the node velocities are taken
%   from the position change of the step, so a correction the constraints
%   or the collision make is not undone by velocity built up before it.

arguments
    haircutState (1, 1) struct
    headCollision = []
    options.HeadClearanceM (1, 1) double {mustBeNonnegative} = 0.001
    options.StrandIndices = []
    options.BendStiffness (1, 1) double {mustBeNonnegative} = 0.05
    options.LengthConstraintStiffness (1, 1) double {mustBeNonnegative} = 0.85
    options.ConstraintIterations (1, 1) double {mustBeInteger, mustBePositive} = 10
end

hairState = haircutState.hairState;
[nodesPerStrand, numStrands] = size(hairState.hairs.X);
strandIndices = hairsim.hair.dynamics.createStrandIndexList(numStrands, options.StrandIndices);
X0 = hairState.hairs.X(:, strandIndices);
Y0 = hairState.hairs.Y(:, strandIndices);
Z0 = hairState.hairs.Z(:, strandIndices);

hairState = hairsim.hair.dynamics.stepGuideHairPhysics(hairState, ...
    'Gravity', haircutState.params.gravity, ...
    'Rigidity', haircutState.params.rigidity, ...
    'Damping', haircutState.params.damping, ...
    'Dt', haircutState.params.dt, ...
    'BendStiffness', options.BendStiffness, ...
    'LengthConstraintStiffness', options.LengthConstraintStiffness, ...
    'ConstraintIterations', options.ConstraintIterations, ...
    'StrandIndices', strandIndices);

if ~isempty(headCollision)
    hairState = hairsim.hair.dynamics.applyGuideHairHeadMeshClearance( ...
        hairState, headCollision, options.HeadClearanceM, 'StrandIndices', strandIndices);
end

% velocity from the position change of the step (active nodes, root excepted)
active = (1:nodesPerStrand)' <= min(hairState.activeNodeCounts(strandIndices), nodesPerStrand);
active(1, :) = false;
scale = haircutState.params.damping / haircutState.params.dt;
hairState.vel.X(:, strandIndices) = (hairState.hairs.X(:, strandIndices) - X0) * scale .* active;
hairState.vel.Y(:, strandIndices) = (hairState.hairs.Y(:, strandIndices) - Y0) * scale .* active;
hairState.vel.Z(:, strandIndices) = (hairState.hairs.Z(:, strandIndices) - Z0) * scale .* active;
haircutState.hairState = hairState;

haircutState.lineData = hairsim.hair.render.createHaircutLineData( ...
    haircutState.hairState, haircutState.referenceHairNodeCounts, ...
    haircutState.debrisList);
end
