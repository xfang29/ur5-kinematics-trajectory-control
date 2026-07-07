function [ J ] = urJacobian(q,type)
% Input: q - 6x1 Joint Angle Vector; type - 'UR5' or 'UR5e'
% Output: g - 4x4 Homogeneous Transformation Matrix

if strcmp(type, 'ur5e')
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
    a = [0, -0.425, -0.3922, 0, 0, 0];
else
    d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
    a = [0, -0.425, -0.39225, 0, 0, 0];
end
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
q_adj = q;
q_adj(1) = q(1) + pi;

T = cell(1,7);
T{1} = eye(4);
for i = 1:6;
    T{i+1} = T{i} * DH(a(i),alpha(i),d(i),q_adj(i));
end

J = zeros(6,6);
p_end = T{7}(1:3,4);

for i = 1:6
    z_prev = T{i}(1:3,3);
    p_prev = T{i}(1:3,4);

    J(1:3,i) = cross(z_prev,(p_end - p_prev));
    J(4:6,i) = z_prev;
end
end