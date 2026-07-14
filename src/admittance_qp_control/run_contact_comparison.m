%% Virtual contact: rigid position control vs admittance control with QP
clear;
clc;
close all;

thisDir = fileparts(mfilename('fullpath'));
srcDir = fileparts(thisDir);
addpath(srcDir);
addpath(thisDir);

cfg = defaultContactConfig();

if exist('quadprog', 'file') ~= 2
    error(['quadprog was not found. Install or enable MATLAB ', ...
           'Optimization Toolbox before running this experiment.']);
end

rigidResult = simulateContactControl('rigid', cfg);
admittanceResult = simulateContactControl('admittance', cfg);

method = ["Rigid + QP"; "Admittance + QP"];

peakForceN = [ ...
    rigidResult.metrics.peakContactForce;
    admittanceResult.metrics.peakContactForce];

maxPenetrationMm = 1e3 * [ ...
    rigidResult.metrics.maxPenetration;
    admittanceResult.metrics.maxPenetration];

contactImpulseNs = [ ...
    rigidResult.metrics.contactImpulse;
    admittanceResult.metrics.contactImpulse];

nominalRmseMm = 1e3 * [ ...
    rigidResult.metrics.nominalPositionRMSE;
    admittanceResult.metrics.nominalPositionRMSE];

referenceRmseMm = 1e3 * [ ...
    rigidResult.metrics.referencePositionRMSE;
    admittanceResult.metrics.referencePositionRMSE];

maxAdmittanceOffsetMm = 1e3 * [ ...
    rigidResult.metrics.maxAdmittanceOffset;
    admittanceResult.metrics.maxAdmittanceOffset];

forceReleaseDelayS = [ ...
    rigidResult.metrics.recovery.forceReleaseDelay;
    admittanceResult.metrics.recovery.forceReleaseDelay];

recoveryAfterReleaseS = [ ...
    rigidResult.metrics.recovery.trackingRecoveryAfterRelease;
    admittanceResult.metrics.recovery.trackingRecoveryAfterRelease];

peakJointSpeed = [ ...
    rigidResult.metrics.peakJointSpeed;
    admittanceResult.metrics.peakJointSpeed];

peakJointAcceleration = [ ...
    rigidResult.metrics.peakJointAcceleration;
    admittanceResult.metrics.peakJointAcceleration];

speedBoundActiveCount = [ ...
    rigidResult.metrics.speedBoundActiveCount;
    admittanceResult.metrics.speedBoundActiveCount];

positionBoundActiveCount = [ ...
    rigidResult.metrics.positionBoundActiveCount;
    admittanceResult.metrics.positionBoundActiveCount];

qpFailureCount = [ ...
    rigidResult.metrics.qpFailureCount;
    admittanceResult.metrics.qpFailureCount];

constraintViolationCount = [ ...
    rigidResult.metrics.constraintViolationCount;
    admittanceResult.metrics.constraintViolationCount];

minimumSigma = [ ...
    rigidResult.metrics.minimumSigma;
    admittanceResult.metrics.minimumSigma];

comparisonTable = table( ...
    method, peakForceN, maxPenetrationMm, contactImpulseNs, ...
    nominalRmseMm, referenceRmseMm, maxAdmittanceOffsetMm, ...
    forceReleaseDelayS, recoveryAfterReleaseS, ...
    peakJointSpeed, peakJointAcceleration, ...
    speedBoundActiveCount, positionBoundActiveCount, ...
    qpFailureCount, constraintViolationCount, minimumSigma);

fprintf('\n=== Virtual contact comparison ===\n');
disp(comparisonTable);

resultsDir = fullfile(thisDir, 'results');
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

save(fullfile(resultsDir, 'contact_comparison.mat'), ...
     'rigidResult', 'admittanceResult', 'comparisonTable');

writetable(comparisonTable, ...
    fullfile(resultsDir, 'contact_comparison_metrics.csv'));

time = rigidResult.time;
retractStart = cfg.approachTime + cfg.pushTime + cfg.holdTime;

figure('Name', 'Contact force');
plot(time, rigidResult.forceMagnitude, 'LineWidth', 1.5);
hold on;
plot(time, admittanceResult.forceMagnitude, '--', 'LineWidth', 1.5);
xline(retractStart, ':', 'Start of return');
grid on;
xlabel('Time (s)');
ylabel('Normal contact force (N)');
title('Virtual contact force');
legend('Rigid + QP', 'Admittance + QP', 'Location', 'best');
exportgraphics(gcf, fullfile(resultsDir, 'contact_force.png'), ...
               'Resolution', 200);

figure('Name', 'Penetration');
plot(time, 1e3 * rigidResult.penetration, 'LineWidth', 1.5);
hold on;
plot(time, 1e3 * admittanceResult.penetration, '--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Virtual penetration (mm)');
title('Penetration into the virtual plane');
legend('Rigid + QP', 'Admittance + QP', 'Location', 'best');
exportgraphics(gcf, fullfile(resultsDir, 'contact_penetration.png'), ...
               'Resolution', 200);

figure('Name', 'Nominal tracking error');
plot(time, 1e3 * rigidResult.nominalPositionError, 'LineWidth', 1.5);
hold on;
plot(time, 1e3 * admittanceResult.nominalPositionError, '--', ...
     'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('||p_{nom} - p||_2 (mm)');
title('Nominal trajectory error');
legend('Rigid + QP', 'Admittance + QP', 'Location', 'best');
exportgraphics(gcf, fullfile(resultsDir, ...
               'contact_nominal_position_error.png'), ...
               'Resolution', 200);

figure('Name', 'Admittance offset');
plot(time, 1e3 * vecnorm(admittanceResult.xAdmittance, 2, 1), ...
     'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('||x_a||_2 (mm)');
title('Admittance-generated reference displacement');
exportgraphics(gcf, fullfile(resultsDir, 'admittance_offset.png'), ...
               'Resolution', 200);

figure('Name', 'Joint speed');
plot(time, max(abs(rigidResult.dq), [], 1), 'LineWidth', 1.5);
hold on;
plot(time, max(abs(admittanceResult.dq), [], 1), '--', ...
     'LineWidth', 1.5);
yline(max(cfg.jointSpeedMax), ':', 'QP speed limit');
grid on;
xlabel('Time (s)');
ylabel('max_i |\dot{q}_i| (rad/s)');
title('QP-constrained joint speed');
legend('Rigid + QP', 'Admittance + QP', 'Location', 'best');
exportgraphics(gcf, fullfile(resultsDir, 'contact_joint_speed.png'), ...
               'Resolution', 200);

figure('Name', 'Force versus penetration');
plot(1e3 * rigidResult.penetration, ...
     rigidResult.forceMagnitude, 'LineWidth', 1.5);
hold on;
plot(1e3 * admittanceResult.penetration, ...
     admittanceResult.forceMagnitude, '--', 'LineWidth', 1.5);
grid on;
xlabel('Virtual penetration (mm)');
ylabel('Normal contact force (N)');
title('Contact force-penetration response');
legend('Rigid + QP', 'Admittance + QP', 'Location', 'best');
exportgraphics(gcf, fullfile(resultsDir, ...
               'contact_force_penetration.png'), ...
               'Resolution', 200);

disp('Comparison complete. Results were saved in the results folder.');
