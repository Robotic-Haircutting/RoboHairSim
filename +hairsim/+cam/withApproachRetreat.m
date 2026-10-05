function [T, phase] = withApproachRetreat(Tcut, n1, nN, approachM, stepM)
%WITHAPPROACHRETREAT CUT poses framed by a straight APPROACH down the first
%   pose's normal and a straight RETREAT up the last one's, constant
%   orientation, every stepM (lead-in / lead-out of the CAM segment).
lifts = (approachM:-stepM:stepM)';
numA = numel(lifts);
numC = size(Tcut, 3);
T = zeros(4, 4, 2 * numA + numC);
phase = strings(2 * numA + numC, 1);
for k = 1:numA
    Ta = Tcut(:, :, 1);
    Ta(1:3, 4) = Ta(1:3, 4) + lifts(k) * n1(:) / norm(n1);
    T(:, :, k) = Ta;
    phase(k) = "APPROACH";
end
T(:, :, numA + (1:numC)) = Tcut;
phase(numA + (1:numC)) = "CUT";
for k = 1:numA
    Tr = Tcut(:, :, end);
    Tr(1:3, 4) = Tr(1:3, 4) + lifts(numA - k + 1) * nN(:) / norm(nN);
    T(:, :, numA + numC + k) = Tr;
    phase(numA + numC + k) = "RETREAT";
end
end
