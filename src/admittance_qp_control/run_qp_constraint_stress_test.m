%% QP constraint stress test
%
% This script deliberately uses a tighter SOFTWARE safety envelope than the
% main contact comparison so that speed and one-step position bounds become
% active and can be verified. These are not manufacturer-rated UR limits.

clear;
clc;
close all;

thisDir = fileparts(mfilename('fullpath'));
srcDir = fileparts(thisDir);
addpath(srcDir);
addpath(thisDir);

cfg = defaultContactConfig();

cfg.jointSpeedMax = 0.02 * ones(6,1);

% Keep the broad software limits for most joints, but deliberately tighten
% joints 1 and 6 around the initial state. With a 0.002-rad margin, the
% effective upper movement allowance is 0.012 rad.
cfg.jointMargin = 0.002 * ones(6,1);
cfg.jointMax(1) = cfg.q0(1) + 0.014;
cfg.jointMax(6) = cfg.q0(6) + 0.014;

rigidResult = simulateContactControl('rigid', cfg);
admittanceResult = simulateContactControl('admittance', cfg);

method = ["Rigid + QP"; "Admittance + QP"];

peakForceN = [rigidResult.metrics.peakContactForce;
              admittanceResult.metrics.peakContactForce];

nominalRmseMm = 1e3 * [rigidResult.metrics.nominalPositionRMSE;
                       admittanceResult.metrics.nominalPositionRMSE];

peakJointSpeed = [rigidResult.metrics.peakJointSpeed;
                  admittanceResult.metrics.peakJointSpeed];

speedBoundActiveCount = [rigidResult.metrics.speedBoundActiveCount;
                         admittanceResult.metrics.speedBoundActiveCount];

positionBoundActiveCount = [rigidResult.metrics.positionBoundActiveCount;
                            admittanceResult.metrics.positionBoundActiveCount];

qpFailureCount = [rigidResult.metrics.qpFailureCount;
                  admittanceResult.metrics.qpFailureCount];

constraintViolationCount = [ ...
    rigidResult.metrics.constraintViolationCount;
    admittanceResult.metrics.constraintViolationCount];

stressTable = table( ...
    method, peakForceN, nominalRmseMm, peakJointSpeed, ...
    speedBoundActiveCount, positionBoundActiveCount, ...
    qpFailureCount, constraintViolationCount);

fprintf('\n=== QP constraint stress test ===\n');
disp(stressTable);

resultsDir = fullfile(thisDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

save(fullfile(resultsDir, 'qp_constraint_stress_test.mat'), ...
     'rigidResult', 'admittanceResult', 'stressTable');

writetable(stressTable, ...
    fullfile(resultsDir, 'qp_constraint_stress_test.csv'));

time = rigidResult.time;

figure('Name', 'Stress-test joint speed');
plot(time, max(abs(rigidResult.dq), [], 1), 'LineWidth', 1.5);
hold on;
plot(time, max(abs(admittanceResult.dq), [], 1), '--', ...
     'LineWidth', 1.5);
yline(max(cfg.jointSpeedMax), ':', 'QP speed limit');
grid on;
xlabel('Time (s)');
ylabel('max_i |\dot{q}_i| (rad/s)');
title('Active joint-speed constraint');
legend('Rigid + QP', 'Admittance + QP', 'Location', 'best');
exportgraphics(gcf, fullfile(resultsDir, ...
               'qp_stress_joint_speed.png'), ...
               'Resolution', 200);

figure('Name', 'Joint 1 position bound');
plot(time, rigidResult.q(1,:), 'LineWidth', 1.5);
hold on;
plot(time, admittanceResult.q(1,:), '--', 'LineWidth', 1.5);
yline(cfg.jointMax(1) - cfg.jointMargin(1), ':', ...
      'Effective QP upper bound');
grid on;
xlabel('Time (s)');
ylabel('q_1 (rad)');
title('One-step joint-position constraint');
legend('Rigid + QP', 'Admittance + QP', 'Location', 'best');
exportgraphics(gcf, fullfile(resultsDir, ...
               'qp_stress_joint_position.png'), ...
               'Resolution', 200);

disp('Stress test complete. Results were saved in the results folder.');
