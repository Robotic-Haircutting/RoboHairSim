function sctx = createStationContext(baseCtx, thetaRad, tool)
%CREATESTATIONCONTEXT Robot context of the camClipper post-processor at
%   one rail station: the rail robot + rail collision objects
%   (baseCtx.railContextBuilder, see hairsim.api.createRobotContext's
%   RailMode), an arm-only copy for self-collision, an arm + clipper-body
%   copy (box + adapter cylinder of hairsim.tool.createClipperTool) so the
%   clipper is part of the self-collision test, and a fresh IK solver.
%
%   Robot links are tested against the head convex hull + environment; the
%   clipper's own clearance separately against the exact head mesh
%   (hairsim.tool.evaluateClipperClearance).

arguments
    baseCtx (1, 1) struct
    thetaRad (1, 1) double
    tool (1, 1) struct
end

%   thetaRad = NaN: no rail -- the context's own fixed base is used as is
%   (a non-rail context offers exactly one "station").
if isnan(thetaRad)
    built = struct('robot', baseCtx.robot, 'headCollision', {baseCtx.headCollision});
else
    if ~isfield(baseCtx, 'railContextBuilder')
        error("hairsim:post:NeedRailContext", ...
            "A rail station was requested but the context has no railContextBuilder ('RailMode', true).");
    end
    built = baseCtx.railContextBuilder(thetaRad);
end
sctx = baseCtx;
sctx.robot = built.robot;
sctx.headCollision = built.headCollision;
sctx.railThetaCurrentRad = thetaRad;
sctx.collisionObjects = built.headCollision;
sctx.armOnlyRobot = built.robot;
sctx.armWithToolRobot = hairsim.tool.createArmWithClipperBody(built.robot, baseCtx.endEffector, tool, baseCtx.qHome);
sctx.ik = inverseKinematics('RigidBodyTree', sctx.robot);
sctx.ik.SolverParameters.AllowRandomRestart = false;
% reused by hairsim.planning.planSafeConfigurationTransition for this
% station, with a cache of its retreat paths (deterministic per start q)
sctx.transitionIk = sctx.ik;
sctx.revoluteJointLimits = hairsim.motion.revoluteJointLimits(sctx.robot);
sctx.retreatCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
% whole transition legs of hairsim.post.postProcessToolpaths (same tool)
sctx.legCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
end
