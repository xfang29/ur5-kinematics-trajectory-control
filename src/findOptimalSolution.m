function [q_sol, idx] = findOptimalSolution(all_solutions, q_current, type)
% FINDOPTIMALSOLUTION - Filter the safest and closest inverse kinematics solution
% Inputs: 
%   all_solutions: urInvKin returned by urInvKin
%   q_current: Current joint angles of the robot (6x1)
%   type: 'ur5' or 'ur5e'
% Outputs:
%   q_sol: Selected 6x1 optimal solution
%   idx: Index of this solution in the original 8 sets of solutions

num_sols = size(all_solutions, 2);
valid_mask = true(1, num_sols);
threshold = 1e-6; % Singularity threshold

for i = 1:num_sols
    q_test = all_solutions(:, i);

    % 1. Check for invalid values (NaN or Complex numbers)
    if any(isnan(q_test)) || any(~isreal(q_test))
        valid_mask(i) = false;
        continue;
    end

    % 2. Singularity check
    % Calculate the Jacobian matrix corresponding to this solution
    J = urJacobian(q_test, type);

    % For a 6-DOF robot like the UR5(e), det(J) approaching 0 indicates a singularity
    if abs(det(J)) < threshold
        valid_mask(i) = false;
        continue;
    end
end

% 3. Find the closest solution among all valid solutions
remaining_indices = find(valid_mask);

if isempty(remaining_indices)
    error('No safe solution found! All possible IK solutions are singular or invalid.');
end

% Compare distances only for valid solutions
valid_solutions = all_solutions(:, remaining_indices);
dists = vecnorm(valid_solutions - q_current, 2, 1);
[~, min_idx] = min(dists);

% Map back to the original index and output
idx = remaining_indices(min_idx);
q_sol = all_solutions(:, idx);
end