%% Closed-loop RR trajectory comparison near a pen-tip Jacobian singularity
%
% This script compares the existing Moore-Penrose pseudoinverse baseline
% against fixed-damping DLS in an offline UR5e simulation.
%
% Both methods use:
%   - the same DH forward kinematics;
%   - the same pen-tip Jacobian;
%   - the same desired line trajectory and Cartesian feedback;
%   - the same 0.23 rad/s whole-vector proportional speed scaling.
%
% The initial q3 = 0.20 rad is nonsingular, but the +Y task drives the
% controller toward an elbow-singular region. This makes the comparison a
% trajectory experiment rather than only an instantaneous matrix test.

clear;
clc;
close all;

thisDir = fileparts(mfilename('fullpath'));
srcDir = fileparts(thisDir);
addpath(srcDir);
addpath(thisDir);

%% Shared experiment configuration
cfg = struct;
cfg.robotType = 'ur5e';
cfg.q0 = [0; -pi/2; 0.20; 0; 1.0; 0];

cfg.direction = [0; 1; 0];
cfg.speed = 0.003;       % m/s
cfg.moveTime = 4.0;      % s
cfg.holdTime = 1.0;      % s
cfg.dt = 0.01;           % s

cfg.Kp = 2.0;            % translational feedback gain [1/s]
cfg.Kr = 2.0;            % rotational feedback gain [1/s]
cfg.lambda = 0.01;       % fixed DLS damping
cfg.jointSpeedLimit = 0.23; % rad/s, same limit as original rr_control.m

cfg.gToolPen = [1, 0, 0, 0;
                0, 1, 0, -0.049;
                0, 0, 1, 0.12228;
                0, 0, 0, 1];

%% Run both controllers
pinvResult = simulateRRNearSingularity('pinv', cfg);
dlsResult = simulateRRNearSingularity('dls', cfg);

%% Print comparison table
method = ["pinv"; "DLS"];
maxErrorMm = 1e3 * [pinvResult.metrics.maxPositionError;
                    dlsResult.metrics.maxPositionError];
rmseMm = 1e3 * [pinvResult.metrics.rmsePosition;
                dlsResult.metrics.rmsePosition];
finalErrorMm = 1e3 * [pinvResult.metrics.finalPositionError;
                      dlsResult.metrics.finalPositionError];
peakRawSpeed = [pinvResult.metrics.peakRawJointSpeed;
                dlsResult.metrics.peakRawJointSpeed];
peakAppliedSpeed = [pinvResult.metrics.peakAppliedJointSpeed;
                    dlsResult.metrics.peakAppliedJointSpeed];
peakAcceleration = [pinvResult.metrics.peakJointAcceleration;
                    dlsResult.metrics.peakJointAcceleration];
velocityVariation = [pinvResult.metrics.jointVelocityTotalVariation;
                     dlsResult.metrics.jointVelocityTotalVariation];
minimumSigma = [pinvResult.metrics.minimumSigma;
                dlsResult.metrics.minimumSigma];
saturationCount = [pinvResult.metrics.saturationCount;
                   dlsResult.metrics.saturationCount];

comparisonTable = table(method, maxErrorMm, rmseMm, finalErrorMm, ...
                        peakRawSpeed, peakAppliedSpeed, ...
                        peakAcceleration, velocityVariation, ...
                        minimumSigma, saturationCount);

fprintf('\n=== Near-singular RR comparison ===\n');
disp(comparisonTable);

%% Save data and CSV
resultsDir = fullfile(thisDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

save(fullfile(resultsDir, 'near_singularity_comparison.mat'), ...
     'pinvResult', 'dlsResult', 'comparisonTable');
writetable(comparisonTable, ...
           fullfile(resultsDir, 'near_singularity_metrics.csv'));

%% Plots
time = pinvResult.time;

figure('Name', 'Position tracking error');
plot(time, 1e3 * pinvResult.positionError, 'LineWidth', 1.5);
hold on;
plot(time, 1e3 * dlsResult.positionError, '--', 'LineWidth', 1.5);
xline(cfg.moveTime, ':', 'End of commanded motion');
grid on;
xlabel('Time (s)');
ylabel('Pen-tip position error (mm)');
title('Near-singular trajectory tracking');
legend('Moore-Penrose pseudoinverse', ...
       sprintf('DLS, \\lambda = %.2f', cfg.lambda), ...
       'Location', 'best');
exportgraphics(gcf, ...
    fullfile(resultsDir, 'trajectory_position_error.png'), ...
    'Resolution', 200);

figure('Name', 'Raw joint-speed demand');
plot(time, max(abs(pinvResult.dqRaw), [], 1), 'LineWidth', 1.5);
hold on;
plot(time, max(abs(dlsResult.dqRaw), [], 1), '--', 'LineWidth', 1.5);
yline(cfg.jointSpeedLimit, ':', 'Applied safety limit');
grid on;
xlabel('Time (s)');
ylabel('max_i |\dot{q}_{i,raw}| (rad/s)');
title('Unconstrained inverse-velocity demand');
legend('Moore-Penrose pseudoinverse', ...
       sprintf('DLS, \\lambda = %.2f', cfg.lambda), ...
       'Location', 'best');
exportgraphics(gcf, ...
    fullfile(resultsDir, 'trajectory_raw_joint_speed.png'), ...
    'Resolution', 200);

figure('Name', 'Applied joint speed');
plot(time, max(abs(pinvResult.dqApplied), [], 1), 'LineWidth', 1.5);
hold on;
plot(time, max(abs(dlsResult.dqApplied), [], 1), '--', 'LineWidth', 1.5);
yline(cfg.jointSpeedLimit, ':', 'Safety limit');
grid on;
xlabel('Time (s)');
ylabel('max_i |\dot{q}_{i}| (rad/s)');
title('Applied joint speed after proportional scaling');
legend('Moore-Penrose pseudoinverse', ...
       sprintf('DLS, \\lambda = %.2f', cfg.lambda), ...
       'Location', 'best');
exportgraphics(gcf, ...
    fullfile(resultsDir, 'trajectory_applied_joint_speed.png'), ...
    'Resolution', 200);

figure('Name', 'Minimum singular value');
semilogy(time, pinvResult.sigmaMin, 'LineWidth', 1.5);
hold on;
semilogy(time, dlsResult.sigmaMin, '--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('\sigma_{min}(J_{pen})');
title('Jacobian conditioning along each closed-loop trajectory');
legend('Moore-Penrose pseudoinverse path', ...
       'DLS path', ...
       'Location', 'best');
exportgraphics(gcf, ...
    fullfile(resultsDir, 'trajectory_sigma_min.png'), ...
    'Resolution', 200);

figure('Name', 'Pen-tip path in XY');
plot(1e3 * pinvResult.pDesired(1,:), ...
     1e3 * pinvResult.pDesired(2,:), 'LineWidth', 2);
hold on;
plot(1e3 * pinvResult.pActual(1,:), ...
     1e3 * pinvResult.pActual(2,:), '--', 'LineWidth', 1.5);
plot(1e3 * dlsResult.pActual(1,:), ...
     1e3 * dlsResult.pActual(2,:), ':', 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('Base X (mm)');
ylabel('Base Y (mm)');
title('Desired and actual pen-tip paths');
legend('Desired', 'Pseudoinverse', 'DLS', 'Location', 'best');
exportgraphics(gcf, ...
    fullfile(resultsDir, 'trajectory_xy_path.png'), ...
    'Resolution', 200);

disp('Comparison complete. Figures, MAT data, and CSV metrics were saved.');
