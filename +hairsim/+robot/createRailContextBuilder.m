function builder = createRailContextBuilder(adapter, headScene, options)
%CREATERAILCONTEXTBUILDER Closure factory for a circular-floor-rail base:
%   builder(thetaRad) -> struct('robot', ..., 'headCollision', ...) at that
%   rail station, everything else (toolMount, headMeshSafetyWorld, qHome,
%   ...) unchanged.
%
%   Planners only ever call ctx.railContextBuilder(thetaRad); this factory
%   is the one place that builds a rail context.
%
%   adapter: hairsim.robot.createRobotAdapter's result (needs plainRobot,
%   endEffector, toolMount, and the fixed-station headCollision to combine
%   with the rail hardware's own collision primitives). headScene:
%   hairsim.createDefaultHeadScene's result.
%
%   options.RadiusM (default 0.60), options.BaseOffsetM (default
%   [0, 0, -0.04], added to the head origin to get the rail's vertical
%   axis) match the values matlab_ur5e's rail-placement study settled on.
%
%   The returned closure is pure (never mutates adapter/headScene) and
%   safe to call speculatively during planning, exactly like the original.

arguments
    adapter (1, 1) struct
    headScene (1, 1) struct
    options.RadiusM (1, 1) double {mustBePositive} = 0.60
    options.BaseOffsetM (1, 3) double {mustBeFinite} = [0, 0, -0.04]
end

headOriginM = headScene.T_world_head(1:3, 4)';
railAxisWorldM = headOriginM + options.BaseOffsetM;
fixedHeadCollision = adapter.headCollision;
plainRobot = adapter.plainRobot;
endEffector = adapter.endEffector;
toolMount = adapter.toolMount;
radiusM = options.RadiusM;

builder = @(thetaRad) buildAt(thetaRad);

    function built = buildAt(thetaRad)
        T_world_base = hairsim.motion.createRailBaseTransform(railAxisWorldM, thetaRad, radiusM);
        env = hairsim.motion.createEnvironmentCollisionPrimitives( ...
            headScene.T_world_head, headScene.headMeshLocal, headScene.headScaleFactor, T_world_base);
        built = struct();
        built.headCollision = [fixedHeadCollision, env.collisionObjects];
        built.robot = hairsim.motion.applyRobotBaseTransform(plainRobot, T_world_base);
    end
end
