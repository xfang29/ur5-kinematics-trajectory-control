function result = simulateRRNearSingularity(method, cfg)
%SIMULATERRNEARSINGULARITY Offline pen-tip RR trajectory simulation.
%
%   result = simulateRRNearSingularity(method, cfg)
%
% Supported methods:
%   'pinv' : Moore-Penrose pseudoinverse using MATLAB pinv
%   'dls'  : damped least-squares inverse
%
% This simulation reuses the existing UR5/UR5e DH forward kinematics and
% pen-tip geometric Jacobian. It does not instantiate ur_interface and does
% not send commands to RViz, URSim, or a physical robot.
%
% The desired task is a straight-line pen-tip motion with fixed orientation.
% A Cartesian feedback term is added identically to both inverse methods:
%
%   V_cmd = [v_ff + Kp (p_d - p);
%            Kr log(R_d R^T)]
%
% The same whole-vector proportional velocity scaling used by the original
% rr_control.m is applied to both methods.

    arguments
        method (1,:) char
        cfg struct
    end

    requiredFields = { ...
        'robotType', 'q0', 'direction', 'speed', 'moveTime', 'holdTime', ...
        'dt', 'Kp', 'Kr', 'lambda', 'jointSpeedLimit', 'gToolPen'};

    for i = 1:numel(requiredFields)
        if ~isfield(cfg, requiredFields{i})
            error('simulateRRNearSingularity:MissingField', ...
                  'Missing cfg.%s.', requiredFields{i});
        end
    end

    method = lower(strtrim(method));
    if ~ismember(method, {'pinv', 'dls'})
        error('simulateRRNearSingularity:UnknownMethod', ...
              'method must be ''pinv'' or ''dls''.');
    end

    q = cfg.q0(:);
    direction = cfg.direction(:);
    direction = direction / norm(direction);

    time = 0:cfg.dt:(cfg.moveTime + cfg.holdTime);
    n = numel(time);

    qLog = zeros(6, n);
    dqRawLog = zeros(6, n);
    dqAppliedLog = zeros(6, n);
    pDesiredLog = zeros(3, n);
    pActualLog = zeros(3, n);
    positionErrorLog = zeros(1, n);
    orientationErrorLog = zeros(1, n);
    sigmaMinLog = zeros(1, n);
    conditionNumberLog = zeros(1, n);
    taskResidualRawLog = zeros(1, n);
    taskResidualAppliedLog = zeros(1, n);
    saturationLog = false(1, n);

    gBaseTool0 = urFwdKin(q, cfg.robotType);
    gBasePen0 = gBaseTool0 * cfg.gToolPen;
    p0 = gBasePen0(1:3, 4);
    RDesired = gBasePen0(1:3, 1:3);

    for k = 1:n
        t = time(k);
        trajectoryTime = min(t, cfg.moveTime);

        pDesired = p0 + direction * cfg.speed * trajectoryTime;
        if t < cfg.moveTime
            vFeedforward = direction * cfg.speed;
        else
            vFeedforward = zeros(3, 1);
        end

        gBaseTool = urFwdKin(q, cfg.robotType);
        gBasePen = gBaseTool * cfg.gToolPen;
        pActual = gBasePen(1:3, 4);
        RActual = gBasePen(1:3, 1:3);

        positionError = pDesired - pActual;

        % J_pen is a geometric Jacobian expressed in the base frame, so use
        % the spatial orientation error log(R_desired * R_actual').
        orientationErrorVector = so3LogVector(RDesired * RActual.');

        VCommand = [vFeedforward + cfg.Kp * positionError;
                    cfg.Kr * orientationErrorVector];

        JPen = calculatePenJacobian(q, pActual, cfg.robotType);
        singularValues = svd(JPen);

        switch method
            case 'pinv'
                dqRaw = pinv(JPen) * VCommand;
            case 'dls'
                dqRaw = dlsJointVelocity(JPen, VCommand, cfg.lambda);
        end

        maxAbsRaw = max(abs(dqRaw));
        if maxAbsRaw > cfg.jointSpeedLimit
            scale = cfg.jointSpeedLimit / maxAbsRaw;
            dqApplied = scale * dqRaw;
            saturationLog(k) = true;
        else
            dqApplied = dqRaw;
        end

        qLog(:, k) = q;
        dqRawLog(:, k) = dqRaw;
        dqAppliedLog(:, k) = dqApplied;
        pDesiredLog(:, k) = pDesired;
        pActualLog(:, k) = pActual;
        positionErrorLog(k) = norm(positionError);
        orientationErrorLog(k) = norm(orientationErrorVector);
        sigmaMinLog(k) = singularValues(end);
        conditionNumberLog(k) = singularValues(1) / singularValues(end);
        taskResidualRawLog(k) = norm(JPen * dqRaw - VCommand);
        taskResidualAppliedLog(k) = norm(JPen * dqApplied - VCommand);

        if k < n
            q = q + dqApplied * cfg.dt;
        end
    end

    jointAcceleration = diff(dqAppliedLog, 1, 2) / cfg.dt;
    dqVariation = diff(dqAppliedLog, 1, 2);

    metrics = struct;
    metrics.maxPositionError = max(positionErrorLog);
    metrics.rmsePosition = sqrt(mean(positionErrorLog.^2));
    metrics.finalPositionError = positionErrorLog(end);
    metrics.maxOrientationError = max(orientationErrorLog);
    metrics.peakRawJointSpeed = max(abs(dqRawLog), [], 'all');
    metrics.peakAppliedJointSpeed = max(abs(dqAppliedLog), [], 'all');
    metrics.peakJointAcceleration = max(abs(jointAcceleration), [], 'all');
    metrics.jointVelocityTotalVariation = ...
        sum(vecnorm(dqVariation, 2, 1));
    metrics.minimumSigma = min(sigmaMinLog);
    metrics.maximumConditionNumber = max(conditionNumberLog);
    metrics.saturationCount = nnz(saturationLog);
    metrics.saturationFraction = nnz(saturationLog) / n;
    metrics.maxRawTaskResidual = max(taskResidualRawLog);
    metrics.maxAppliedTaskResidual = max(taskResidualAppliedLog);

    result = struct;
    result.method = method;
    result.cfg = cfg;
    result.time = time;
    result.q = qLog;
    result.dqRaw = dqRawLog;
    result.dqApplied = dqAppliedLog;
    result.pDesired = pDesiredLog;
    result.pActual = pActualLog;
    result.positionError = positionErrorLog;
    result.orientationError = orientationErrorLog;
    result.sigmaMin = sigmaMinLog;
    result.conditionNumber = conditionNumberLog;
    result.taskResidualRaw = taskResidualRawLog;
    result.taskResidualApplied = taskResidualAppliedLog;
    result.saturated = saturationLog;
    result.metrics = metrics;
end
