%% DLS damping parameter sweep
clear;
clc;
close all;

thisDir = fileparts(mfilename('fullpath'));
srcDir = fileparts(thisDir);
addpath(srcDir);
addpath(thisDir);

cfg = struct;
cfg.robotType = 'ur5e';
cfg.q0 = [0; -pi/2; 0.20; 0; 1.0; 0];

cfg.direction = [0; 1; 0];
cfg.speed = 0.003;
cfg.moveTime = 4.0;
cfg.holdTime = 1.0;
cfg.dt = 0.01;

cfg.Kp = 2.0;
cfg.Kr = 2.0;
cfg.jointSpeedLimit = 0.23;

cfg.gToolPen = [1, 0, 0, 0;
                0, 1, 0, -0.049;
                0, 0, 1, 0.12228;
                0, 0, 0, 1];

lambdaValues = [0.005; 0.01; 0.02; 0.03; 0.05];
n = numel(lambdaValues);

maxErrorMm = zeros(n,1);
rmseMm = zeros(n,1);
finalErrorMm = zeros(n,1);
maxOrientationDeg = zeros(n,1);
peakRawSpeed = zeros(n,1);
peakAppliedSpeed = zeros(n,1);
peakAcceleration = zeros(n,1);
velocityVariation = zeros(n,1);
minimumSigma = zeros(n,1);
saturationCount = zeros(n,1);

allResults = cell(n,1);

for k = 1:n
    cfg.lambda = lambdaValues(k);

    result = simulateRRNearSingularity('dls', cfg);
    allResults{k} = result;

    maxErrorMm(k) = 1000 * result.metrics.maxPositionError;
    rmseMm(k) = 1000 * result.metrics.rmsePosition;
    finalErrorMm(k) = 1000 * result.metrics.finalPositionError;

    maxOrientationDeg(k) = ...
        rad2deg(result.metrics.maxOrientationError);

    peakRawSpeed(k) = result.metrics.peakRawJointSpeed;
    peakAppliedSpeed(k) = result.metrics.peakAppliedJointSpeed;
    peakAcceleration(k) = result.metrics.peakJointAcceleration;

    velocityVariation(k) = ...
        result.metrics.jointVelocityTotalVariation;

    minimumSigma(k) = result.metrics.minimumSigma;
    saturationCount(k) = result.metrics.saturationCount;
end

lambdaTable = table( ...
    lambdaValues, ...
    maxErrorMm, ...
    rmseMm, ...
    finalErrorMm, ...
    maxOrientationDeg, ...
    peakRawSpeed, ...
    peakAppliedSpeed, ...
    peakAcceleration, ...
    velocityVariation, ...
    minimumSigma, ...
    saturationCount);

disp(lambdaTable);

resultsDir = fullfile(thisDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

writetable(lambdaTable, ...
    fullfile(resultsDir, 'dls_lambda_sweep.csv'));

save(fullfile(resultsDir, 'dls_lambda_sweep.mat'), ...
     'lambdaTable', 'allResults');

figure;
semilogx(lambdaValues, rmseMm, '-o', 'LineWidth', 1.5);
grid on;
xlabel('\lambda');
ylabel('Position RMSE (mm)');
title('DLS tracking error versus damping');

figure;
semilogx(lambdaValues, peakRawSpeed, '-o', 'LineWidth', 1.5);
hold on;
yline(cfg.jointSpeedLimit, '--', 'Safety limit');
grid on;
xlabel('\lambda');
ylabel('Peak raw joint speed (rad/s)');
title('DLS joint-speed demand versus damping');

figure;
semilogx(lambdaValues, peakAcceleration, '-o', 'LineWidth', 1.5);
grid on;
xlabel('\lambda');
ylabel('Peak finite-difference acceleration (rad/s^2)');
title('DLS smoothness versus damping');