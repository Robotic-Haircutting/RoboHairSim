function tool = createClipperTool(options)
%CREATECLIPPERTOOL The hair clipper as the single tool model of the
%   camClipper pipeline (+cam toolpaths, +post robot post-processing,
%   +eval coverage, comb-lift cutting), of the Teleop and of every
%   clearance / collision gate (hairsim.motion.createClipperToolMount).
%
%   Frames. EE frame:
%   x = feed direction, y = blade width, z = toward the scalp (-n). The TCP
%   frame has the same axes, origin at the centre of the teeth face:
%       T_ee_tcp = translation [0, 0, tcpOffsetM].
%   In TCP coordinates the scalp lies at +z; a planned pose puts the TCP at
%   scalpPoint + nominalStandoffM * n_out.
%
%   Geometry is parametric, sized from the measured head of data/HairClipper.stl
%   as mounted by hairsim.motion.createClipperDisplayMesh (teeth face 25-26 mm
%   wide x 5-6 mm thick, still <= 7 mm thick 5 mm back, 15 mm thick 15 mm
%   back, teeth at EE z = 45 mm). The STL itself is not used (unlicensed).
%     contact slab  the part that meets the scalp: |x| <= contactHalfThicknessM,
%                   |y| <= bladeHalfWidthM, z in [-contactDepthM, 0] (TCP).
%                   Checked against the EXACT head mesh (signed distance).
%     body          wedge upper part (box) + adapter (cylinder to the flange).
%                   Also checked against the exact head mesh -- never the
%                   convex hull, which bridges the ear/neck concavities.
%
%   Comb-lift cutting (hairsim.hair.cutting.applyCombLiftCutToGuideHair /
%   hairsim.eval.sweepCombLiftCoverage): a strand is stood up and cut at its
%   target length when its ROOT lies in the footprint
%       x in [-contactHalfThicknessM, contactHalfThicknessM + combLeadM],
%       |y| <= bladeHalfWidthM
%   and the blade segment above that root is within
%       0 <= localHeight <= nominalStandoffM + contactToleranceM
%   of it (localHeight = the root's TCP z). This one per-root rule covers a
%   centred blade, a laterally offset blade and a rolled blade (edge cut).
%   MaxFeedAngleRad: the comb only lifts hair it is pushed against; when
%   finite, a root also needs angle(feed x, -hair flow) <= this. Inf
%   (default) = direction ignored.

arguments
    options.TcpOffsetM (1, 1) double {mustBePositive} = 0.045
    options.BladeHalfWidthM (1, 1) double {mustBePositive} = 0.013
    options.ContactHalfThicknessM (1, 1) double {mustBePositive} = 0.0035
    options.ContactDepthM (1, 1) double {mustBePositive} = 0.005
    options.CombLeadM (1, 1) double {mustBeNonnegative} = 0.005
    options.NominalStandoffM (1, 1) double {mustBeNonnegative} = 0.002
    options.ContactToleranceM (1, 1) double {mustBeNonnegative} = 0.003
    options.SafetyClearanceM (1, 1) double {mustBeNonnegative} = 0.001
    options.BodyHalfSizeM (1, 2) double {mustBePositive} = [0.008, 0.014]
    options.BodyZRangeM (1, 2) double = [-0.017, -0.005]
    options.AdapterRadiusM (1, 1) double {mustBePositive} = 0.008
    options.EdgeLateralOffsetsM (1, :) double = [0.005, 0.010]
    options.EdgeRollAnglesRad (1, :) double = deg2rad([15, 30])
    options.MaxFeedAngleRad (1, 1) double {mustBePositive} = Inf
end

tool = struct();
tool.name = "clipper";
tool.tcpOffsetM = options.TcpOffsetM;
tool.T_ee_tcp = eye(4);
tool.T_ee_tcp(3, 4) = options.TcpOffsetM;
tool.bladeHalfWidthM = options.BladeHalfWidthM;
tool.contactHalfThicknessM = options.ContactHalfThicknessM;
tool.contactDepthM = options.ContactDepthM;
tool.combLeadM = options.CombLeadM;
tool.nominalStandoffM = options.NominalStandoffM;
tool.contactToleranceM = options.ContactToleranceM;
tool.safetyClearanceM = options.SafetyClearanceM;
tool.footprintXRangeM = [-options.ContactHalfThicknessM, options.ContactHalfThicknessM + options.CombLeadM];
tool.maxLocalHeightM = options.NominalStandoffM + options.ContactToleranceM;
tool.maxFeedAngleRad = options.MaxFeedAngleRad;
% Edge-cut variants (CAM corner cleanup): +-offset along y, +-roll about x.
tool.edgeLateralOffsetsM = options.EdgeLateralOffsetsM;
tool.edgeRollAnglesRad = options.EdgeRollAnglesRad;

% ---- sample points in the TCP frame
hx = options.ContactHalfThicknessM;
hy = options.BladeHalfWidthM;
[cx, cy, cz] = ndgrid([-hx, 0, hx], linspace(-hy, hy, 9), [0, -0.5, -1] * options.ContactDepthM);
tool.contactPointsTcpM = [cx(:), cy(:), cz(:)];

bx = options.BodyHalfSizeM(1);
by = options.BodyHalfSizeM(2);
[px, py, pz] = ndgrid([-bx, 0, bx], linspace(-by, by, 5), linspace(options.BodyZRangeM(2), options.BodyZRangeM(1), 3));
boxPoints = [px(:), py(:), pz(:)];
angles = (0:7)' * (2 * pi / 8);
ringZ = linspace(options.BodyZRangeM(1), -options.TcpOffsetM, 5);
[aa, zz] = ndgrid(angles, ringZ);
adapterPoints = [options.AdapterRadiusM * cos(aa(:)), options.AdapterRadiusM * sin(aa(:)), zz(:)];
tool.bodyPointsTcpM = [boxPoints; adapterPoints];
end
