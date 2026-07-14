function dq = dlsJointVelocity(J, V, lambda)
%DLSJOINTVELOCITY Compute a damped least-squares inverse-velocity solution.
%
%   dq = dlsJointVelocity(J, V, lambda)
%
% Solves
%   min_dq ||J*dq - V||_2^2 + lambda^2 ||dq||_2^2
%
% using the numerically stable form
%   dq = J' * (J*J' + lambda^2 I) \ V
%
% Inputs
%   J       : m-by-n task Jacobian
%   V       : m-by-1 desired task-space velocity
%   lambda  : nonnegative damping coefficient
%
% Output
%   dq      : n-by-1 joint velocity

    arguments
        J double
        V double
        lambda (1,1) double {mustBeNonnegative}
    end

    if size(J, 1) ~= numel(V)
        error('dlsJointVelocity:DimensionMismatch', ...
              'The number of rows of J must equal the length of V.');
    end

    V = V(:);
    taskDim = size(J, 1);
    regularizedMatrix = J * J.' + lambda^2 * eye(taskDim);

    % Do not form an explicit matrix inverse.
    dq = J.' * (regularizedMatrix \ V);
end
