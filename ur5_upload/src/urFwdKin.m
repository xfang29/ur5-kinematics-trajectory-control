function [ g ] = urFwdKin(q,type)
% Input: q - 6x1 Joint Angle Vector; type - 'UR5' or 'UR5e'
% Output: g - 4x4 Homogeneous Transformation Matrix

% [a, alpha, d, theta]
if strcmp(type, 'ur5e')
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
    a = [0, -0.425, -0.3922, 0, 0, 0];
else % ur5
    d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
    a = [0, -0.425, -0.39225, 0, 0, 0];
end
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

q_adj = q;
q_adj(1) = q(1) + pi;

g = eye(4);
for i = 1:6;
    g = g * DH(a(i),alpha(i),d(i),q_adj(i));
end

end
