function sctx = createStationContext(baseCtx, thetaRad, tool)
%CREATESTATIONCONTEXT Robot context of the camClipper post-processor at
%   one rail station: the rail robot + rail collision objects
%   (baseCtx.railContextBuilder, see hairsim.api.createRobotContext's
%   RailMode), an arm-only copy for self-collision, an arm + clipper-body
%   copy (box + adapter cylinder of hairsim.tool.createClipperTool) so the
%   clipper is part of the self-collision test, and a fresh IK solver.
%
%   The robot-vs-head test itself is unchanged (robot links vs the head
%   convex hull + environment, ClipperTool_Link stripped -- the frozen
%   rule); the clipper's own clearance is tested separately against the
%   exact head mesh (hairsim.tool.evaluateClipperClearance).

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
sctx.armOnlyRobot = copy(built.robot);
if any(strcmp(sctx.armOnlyRobot.BodyNames, 'ClipperTool_Link'))
    removeBody(sctx.armOnlyRobot, 'ClipperTool_Link');
end
sctx.armWithToolRobot = attachToolBody(sctx.armOnlyRobot, baseCtx.endEffector, tool, baseCtx.qHome);
sctx.ik = inverseKinematics('RigidBodyTree', sctx.robot);
sctx.ik.SolverParameters.AllowRandomRestart = false;
end

function robot = attachToolBody(armOnly, endEffector, tool, qHome)
% Box over the wedge's upper part; the adapter cylinder starts 10 mm off
% the flange so it never touches the wrist link it is bolted to.
robot = copy(armOnly);
body = rigidBody('ClipperBody_Link');
setFixedTransform(body.Joint, eye(4));
boxZ = tool.tcpOffsetM + mean(tool.bodyPointsTcpM(1:45, 3));
boxSize = [2 * max(tool.bodyPointsTcpM(1:45, 1)), 2 * max(tool.bodyPointsTcpM(1:45, 2)), ...
    max(tool.bodyPointsTcpM(1:45, 3)) - min(tool.bodyPointsTcpM(1:45, 3))];
addCollision(body, "box", boxSize, trvec2tform([0, 0, boxZ]));
cylTop = tool.tcpOffsetM + min(tool.bodyPointsTcpM(1:45, 3));
cylLen = cylTop - 0.010;
addCollision(body, "cylinder", [0.008, cylLen], trvec2tform([0, 0, 0.010 + cylLen / 2]));
addBody(robot, body, char(endEffector));
if checkCollision(robot, qHome(:), 'SkippedSelfCollisions', 'parent')
    warning("hairsim:post:ToolBodySelfCollisionAtHome", ...
        "Clipper body self-collides at Home; self-collision falls back to the arm only.");
    robot = armOnly;
end
end
