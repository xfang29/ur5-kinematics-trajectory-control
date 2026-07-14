%% Pen-tip Jacobian singularity sweep: Moore-Penrose pseudoinverse vs DLS
%
% Place this folder at:
%   <repository>/src/singularity_analysis/
%
% The script automatically adds the parent src folder to the MATLAB path.
% It reuses the existing DH.m, urFwdKin.m, and calculatePenJacobian.m files.
%
% Experiment:
%   q3 is reduced from 0.50 rad toward 0.01 rad while the other joints are
%   fixed. At each configuration, a pure +Y pen-tip velocity is mapped to
%   joint velocity using:
%       1) MATLAB pinv(J_pen)
%       2) fixed-damping DLS
%
% This is an instantaneous inverse-kinematics analysis; no joint state is
% integrated in this script.

clear;
clc;
close all;

thisDir = fileparts(mfilename('fullpath'));
srcDir = fileparts(thisDir);
addpath(srcDir);
addpath(thisDir);

%% Configuration
robotType = 'ur5e';

qTemplate = [0; -pi/2; 0; 0; 1.0; 0];
q3Values = linspace(0.50, 0.01, 250);

taskVelocity = [0; 0.01; 0; 0; 0; 0];  % 1 cm/s along base-frame +Y
lambda = 0.05;

gToolPen = [1, 0, 0, 0;
            0, 1, 0, -0.049;
            0, 0, 1, 0.12228;
            0, 0, 0, 1];

n = numel(q3Values);
sigmaMin = zeros(1, n);
conditionNumber = zeros(1, n);
dqPinv = zeros(6, n);
dqDls = zeros(6, n);
residualPinv = zeros(1, n);
residualDls = zeros(1, n);

%% Sweep
for k = 1:n
    q = qTemplate;
    q(3) = q3Values(k);

    gBaseTool = urFwdKin(q, robotType);
    gBasePen = gBaseTool * gToolPen;
    pPen = gBasePen(1:3, 4);

    JPen = calculatePenJacobian(q, pPen, robotType);
    singularValues = svd(JPen);

    sigmaMin(k) = singularValues(end);
    conditionNumber(k) = singularValues(1) / singularValues(end);

    dqPinv(:, k) = pinv(JPen) * taskVelocity;
    dqDls(:, k) = dlsJointVelocity(JPen, taskVelocity, lambda);

    residualPinv(k) = norm(JPen * dqPinv(:, k) - taskVelocity);
    residualDls(k) = norm(JPen * dqDls(:, k) - taskVelocity);
end

maxJointSpeedPinv = max(abs(dqPinv), [], 1);
maxJointSpeedDls = max(abs(dqDls), [], 1);
jointSpeedNormPinv = vecnorm(dqPinv, 2, 1);
jointSpeedNormDls = vecnorm(dqDls, 2, 1);

%% Console summary
fprintf('\n=== Pen-tip Jacobian singularity sweep ===\n');
fprintf('Robot model: %s\n', robotType);
fprintf('Commanded twist: [0, 0.01, 0, 0, 0, 0]^T\n');
fprintf('DLS lambda: %.4f\n', lambda);
fprintf('Minimum sigma_min in sweep: %.6e\n', min(sigmaMin));
fprintf('Maximum condition number: %.6e\n', max(conditionNumber));
fprintf('Peak max|dq_i| with pinv: %.6f rad/s\n', max(maxJointSpeedPinv));
fprintf('Peak max|dq_i| with DLS : %.6f rad/s\n', max(maxJointSpeedDls));
fprintf('Maximum DLS task residual: %.6e\n', max(residualDls));
fprintf('==========================================\n\n');

%% Plots
resultsDir = fullfile(thisDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

figure('Name', 'Minimum singular value');
plot(q3Values, sigmaMin, 'LineWidth', 1.5);
grid on;
xlabel('q_3 (rad)');
ylabel('\sigma_{min}(J_{pen})');
title('Approach to the elbow singularity');
set(gca, 'XDir', 'reverse');
exportgraphics(gcf, fullfile(resultsDir, 'sweep_sigma_min.png'), ...
               'Resolution', 200);

figure('Name', 'Joint-speed amplification');
semilogy(q3Values, maxJointSpeedPinv, 'LineWidth', 1.5);
hold on;
semilogy(q3Values, maxJointSpeedDls, '--', 'LineWidth', 1.5);
grid on;
xlabel('q_3 (rad)');
ylabel('max_i |\dot{q}_i| (rad/s)');
title('Near-singular joint-speed amplification');
legend('Moore-Penrose pseudoinverse', ...
       sprintf('DLS, \\lambda = %.2f', lambda), ...
       'Location', 'best');
set(gca, 'XDir', 'reverse');
exportgraphics(gcf, fullfile(resultsDir, 'sweep_joint_speed.png'), ...
               'Resolution', 200);

figure('Name', 'Task-space residual');
semilogy(q3Values, max(residualPinv, eps), 'LineWidth', 1.5);
hold on;
semilogy(q3Values, max(residualDls, eps), '--', 'LineWidth', 1.5);
grid on;
xlabel('q_3 (rad)');
ylabel('||J\dot{q} - V_{cmd}||_2');
title('Task-space velocity residual');
legend('Moore-Penrose pseudoinverse', ...
       sprintf('DLS, \\lambda = %.2f', lambda), ...
       'Location', 'best');
set(gca, 'XDir', 'reverse');
exportgraphics(gcf, fullfile(resultsDir, 'sweep_task_residual.png'), ...
               'Resolution', 200);

%% Save reproducible data
results = struct;
results.robotType = robotType;
results.qTemplate = qTemplate;
results.q3Values = q3Values;
results.taskVelocity = taskVelocity;
results.lambda = lambda;
results.sigmaMin = sigmaMin;
results.conditionNumber = conditionNumber;
results.dqPinv = dqPinv;
results.dqDls = dqDls;
results.maxJointSpeedPinv = maxJointSpeedPinv;
results.maxJointSpeedDls = maxJointSpeedDls;
results.jointSpeedNormPinv = jointSpeedNormPinv;
results.jointSpeedNormDls = jointSpeedNormDls;
results.residualPinv = residualPinv;
results.residualDls = residualDls;

save(fullfile(resultsDir, 'singularity_sweep.mat'), 'results');

disp('Sweep complete. Figures and MAT data were saved in the results folder.');
