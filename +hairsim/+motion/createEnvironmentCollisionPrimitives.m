function env = createEnvironmentCollisionPrimitives(T_world_head, headMeshLocal, headScaleFactor, T_world_base, options)
%CREATEENVIRONMENTCOLLISIONPRIMITIVES Simplified floor / torso / shoulder obstacles.
%   A minimal "body" model so that moving the robot base around the head cannot
%   find solutions that pass through the patient's body or the floor. It is a
%   simplified collision model, not anatomy: two primitives plus a floor.
%
%   Frames (all rigid, so translating/rotating head+base together moves everything):
%     torso, shoulders  bound to the HEAD frame:  T_world_prim = T_world_head * T_head_prim
%     floor             bound to the BASE mount:  z_floor = z_base - FloorBelowBaseM
%   The head frame's axes are aligned with the anatomy for HeadScan.stl (checked
%   from the scalp_left/right/back region labels): local +z up, local x is the
%   left-right axis, local +y is the face direction. Torso and shoulder box are
%   therefore vertical / width-along-x in the head frame.
%
%   Geometry (defaults):
%     torso     cylinder r = 0.16 m, h = 0.50 m, axis through the head-centre XY,
%               top TorsoTopBelowHeadM = 0.04 m below the lowest head vertex; the
%               height is cut at the floor (it never pokes through it).
%     shoulders box 0.40 (x, width) x 0.22 (y, depth) x 0.12 (z) m, top flush
%               with the torso top, centred on the torso axis.
%     floor     thin slab whose TOP face is at z_base - FloorBelowBaseM (the base
%               mounting surface is assumed FloorBelowBaseM = 0.15 m above the
%               floor -- a simulation assumption, there is no physical stand).
%   env.collisionObjects are INFLATED by InflateM so that the planner's single
%   separation margin (0.001 m for the head) gives an effective 0.020 m clearance
%   to the environment; env.display holds the un-inflated geometry.
%
%   env.collisionObjects  1x3 cell {floor, torso, shoulders}, world frame
%   env.names             ["FLOOR" "TORSO" "SHOULDERS"]
%   env.display           struct array: name, type, T_world_prim, size (box [x y z] or
%                         cylinder [radius length]) of the true (un-inflated) shapes

arguments
    T_world_head (4, 4) double {mustBeFinite}
    headMeshLocal (1, 1) struct
    headScaleFactor (1, 1) double {mustBePositive}
    T_world_base (4, 4) double {mustBeFinite}
    options.TorsoRadiusM (1, 1) double {mustBePositive} = 0.16
    options.TorsoHeightM (1, 1) double {mustBePositive} = 0.50
    options.TorsoTopBelowHeadM (1, 1) double {mustBeNonnegative} = 0.04
    options.ShoulderSizeM (1, 3) double {mustBePositive} = [0.40, 0.22, 0.12]
    options.FloorBelowBaseM (1, 1) double {mustBePositive} = 0.15
    options.FloorSlabSizeM (1, 3) double {mustBePositive} = [6, 6, 0.10]
    options.InflateM (1, 1) double {mustBeNonnegative} = 0.019
end

vLocal = headMeshLocal.vertices * headScaleFactor;
centerXyLocal = 0.5 * (min(vLocal(:, 1:2), [], 1) + max(vLocal(:, 1:2), [], 1));
torsoTopLocalZ = min(vLocal(:, 3)) - options.TorsoTopBelowHeadM;

zFloor = T_world_base(3, 4) - options.FloorBelowBaseM;
torsoTopWorldZ = T_world_head(3, 4) + T_world_head(3, 3) * torsoTopLocalZ;
torsoHeightM = min(options.TorsoHeightM, torsoTopWorldZ - zFloor);
if torsoHeightM <= 0
    error("hairsim:TorsoBelowFloor", ...
        "Torso top (z=%.3f) is at or below the floor (z=%.3f).", torsoTopWorldZ, zFloor);
end

i = options.InflateM;
sh = options.ShoulderSizeM;

% torso: axis along head-local z
T_head_torso = trvec2tform([centerXyLocal, torsoTopLocalZ - torsoHeightM / 2]);
T_world_torso = T_world_head * T_head_torso;
% shoulders: top flush with the torso top
T_head_sh = trvec2tform([centerXyLocal, torsoTopLocalZ - sh(3) / 2]);
T_world_sh = T_world_head * T_head_sh;
% floor: horizontal slab under the head, top face at zFloor
slab = options.FloorSlabSizeM;
T_world_floor = trvec2tform([T_world_head(1, 4), T_world_head(2, 4), zFloor - slab(3) / 2]);

floorObj = collisionBox(slab(1) + 2 * i, slab(2) + 2 * i, slab(3) + i);
floorObj.Pose = trvec2tform([T_world_head(1, 4), T_world_head(2, 4), zFloor + i - (slab(3) + i) / 2]);
torsoObj = collisionCylinder(options.TorsoRadiusM + i, torsoHeightM + 2 * i);
torsoObj.Pose = T_world_torso;
shoulderObj = collisionBox(sh(1) + 2 * i, sh(2) + 2 * i, sh(3) + 2 * i);
shoulderObj.Pose = T_world_sh;

env = struct();
env.collisionObjects = {floorObj, torsoObj, shoulderObj};
env.names = ["FLOOR", "TORSO", "SHOULDERS"];
env.display = struct( ...
    'name', {"FLOOR", "TORSO", "SHOULDERS"}, ...
    'type', {"box", "cylinder", "box"}, ...
    'T_world_prim', {T_world_floor, T_world_torso, T_world_sh}, ...
    'size', {slab, [options.TorsoRadiusM, torsoHeightM], sh});
env.zFloorM = zFloor;
env.torsoHeightM = torsoHeightM;
env.inflateM = i;
end
