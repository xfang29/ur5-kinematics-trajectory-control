% Calculate Pen Tip Jacobian
function Jp = calculatePenJacobian(q, p_pen, type)
if strcmp(type, 'ur5e')
    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
    a = [0, -0.425, -0.3922, 0, 0, 0];
else
    d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
    a = [0, -0.425, -0.39225, 0, 0, 0];
end
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
q_adj = q; q_adj(1) = q(1) + pi;

T = cell(1,7); T{1} = eye(4);
for i = 1:6
    T{i+1} = T{i} * DH(a(i), alpha(i), d(i), q_adj(i));
end

Jp = zeros(6,6);
for i = 1:6
    z_prev = T{i}(1:3,3);
    p_prev = T{i}(1:3,4);
    Jp(1:3,i) = cross(z_prev, (p_pen - p_prev));
    Jp(4:6,i) = z_prev;
end
end