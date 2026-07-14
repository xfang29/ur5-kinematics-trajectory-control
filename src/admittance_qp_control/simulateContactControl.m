function result = simulateContactControl(mode, cfg)
%SIMULATECONTACTCONTROL Offline rigid/admittance contact simulation.
%
% mode:
%   'rigid'      - no admittance displacement; tracks nominal trajectory
%   'admittance' - Cartesian mass-damper-spring reference modification
%
% Both modes use the same velocity-level QP and the same virtual contact
% model so the comparison isolates the compliance contribution.

    arguments
        mode (1,:) char
        cfg struct
    end

    mode = lower(strtrim(mode));
    if ~ismember(mode, {'rigid', 'admittance'})
        error('simulateContactControl:UnknownMode', ...
            'mode must be ''rigid'' or ''admittance''.');
    end

    q = cfg.q0(:);
    dqPrevious = zeros(6,1);

    gBaseTool0 = urFwdKin(q, cfg.robotType);
    gBasePen0 = gBaseTool0 * cfg.gToolPen;

    p0 = gBasePen0(1:3,4);
    RDesired = gBasePen0(1:3,1:3);

    n = cfg.contactNormal(:);
    n = n / norm(n);
    planePoint = p0 + n * cfg.surfaceDistance;

    totalTime = cfg.approachTime + cfg.pushTime + cfg.holdTime ...
              + cfg.returnTime + cfg.settleTime;

    time = 0:cfg.dt:totalTime;
    sampleCount = numel(time);

    xAdmittance = zeros(3,1);
    vAdmittance = zeros(3,1);

    qLog = zeros(6, sampleCount);
    dqLog = zeros(6, sampleCount);
    pNominalLog = zeros(3, sampleCount);
    pReferenceLog = zeros(3, sampleCount);
    pActualLog = zeros(3, sampleCount);
    xAdmittanceLog = zeros(3, sampleCount);
    vAdmittanceLog = zeros(3, sampleCount);
    forceVectorLog = zeros(3, sampleCount);
    forceMagnitudeLog = zeros(1, sampleCount);
    penetrationLog = zeros(1, sampleCount);
    nominalPositionErrorLog = zeros(1, sampleCount);
    referencePositionErrorLog = zeros(1, sampleCount);
    orientationErrorLog = zeros(1, sampleCount);
    sigmaMinLog = zeros(1, sampleCount);
    taskResidualLog = zeros(1, sampleCount);
    qpExitFlagLog = zeros(1, sampleCount);
    qpFallbackLog = false(1, sampleCount);
    speedBoundActiveLog = false(1, sampleCount);
    positionBoundActiveLog = false(1, sampleCount);
    speedViolationLog = false(1, sampleCount);
    positionViolationLog = false(1, sampleCount);
    phaseLog = strings(1, sampleCount);

    for k = 1:sampleCount
        t = time(k);

        gBaseTool = urFwdKin(q, cfg.robotType);
        gBasePen = gBaseTool * cfg.gToolPen;

        pActual = gBasePen(1:3,4);
        RActual = gBasePen(1:3,1:3);

        JPen = calculatePenJacobian(q, pActual, cfg.robotType);
        vPenEstimate = JPen(1:3,:) * dqPrevious;

        contact = virtualPlaneContact( ...
            pActual, vPenEstimate, planePoint, cfg);

        if strcmp(mode, 'admittance')
            aAdmittance = cfg.admittanceMass \ ( ...
                contact.forceVector ...
                - cfg.admittanceDamping * vAdmittance ...
                - cfg.admittanceStiffness * xAdmittance);

            % Semi-implicit Euler integration.
            vAdmittance = ...
                vAdmittance + cfg.dt * aAdmittance;
            xAdmittance = ...
                xAdmittance + cfg.dt * vAdmittance;
        else
            xAdmittance(:) = 0;
            vAdmittance(:) = 0;
        end

        [pNominal, vNominal, phase] = ...
            contactNominalTrajectory(t, p0, cfg);

        pReference = pNominal + xAdmittance;

        positionFeedback = cfg.Kp * (pReference - pActual);
        orientationErrorVector = ...
            rotationLogVector(RDesired * RActual.');

        Vcmd = [vNominal + vAdmittance + positionFeedback;
                cfg.Kr * orientationErrorVector];

        [dq, qpInfo] = solveVelocityQP( ...
            JPen, Vcmd, q, dqPrevious, cfg);

        qLog(:,k) = q;
        dqLog(:,k) = dq;
        pNominalLog(:,k) = pNominal;
        pReferenceLog(:,k) = pReference;
        pActualLog(:,k) = pActual;
        xAdmittanceLog(:,k) = xAdmittance;
        vAdmittanceLog(:,k) = vAdmittance;
        forceVectorLog(:,k) = contact.forceVector;
        forceMagnitudeLog(k) = contact.forceMagnitude;
        penetrationLog(k) = contact.penetration;
        nominalPositionErrorLog(k) = norm(pNominal - pActual);
        referencePositionErrorLog(k) = norm(pReference - pActual);
        orientationErrorLog(k) = norm(orientationErrorVector);
        sigmaMinLog(k) = min(svd(JPen));
        taskResidualLog(k) = norm(JPen * dq - Vcmd);
        qpExitFlagLog(k) = qpInfo.exitFlag;
        qpFallbackLog(k) = qpInfo.usedFallback;
        speedBoundActiveLog(k) = qpInfo.speedBoundActive;
        positionBoundActiveLog(k) = qpInfo.positionBoundActive;
        speedViolationLog(k) = qpInfo.speedViolation;
        positionViolationLog(k) = qpInfo.positionViolation;
        phaseLog(k) = string(phase);

        if k < sampleCount
            q = q + dq * cfg.dt;
            dqPrevious = dq;
        end
    end

    jointAcceleration = diff(dqLog, 1, 2) / cfg.dt;

    recovery = computeRecoveryMetrics( ...
        time, forceMagnitudeLog, nominalPositionErrorLog, cfg);

    metrics = struct;
    metrics.peakContactForce = max(forceMagnitudeLog);
    metrics.maxPenetration = max(penetrationLog);
    metrics.contactImpulse = trapz(time, forceMagnitudeLog);
    metrics.nominalPositionRMSE = ...
        sqrt(mean(nominalPositionErrorLog.^2));
    metrics.referencePositionRMSE = ...
        sqrt(mean(referencePositionErrorLog.^2));
    metrics.maxAdmittanceOffset = ...
        max(vecnorm(xAdmittanceLog, 2, 1));
    metrics.finalAdmittanceOffset = ...
        norm(xAdmittanceLog(:,end));
    metrics.maxOrientationError = max(orientationErrorLog);
    metrics.peakJointSpeed = max(abs(dqLog), [], 'all');
    metrics.peakJointAcceleration = ...
        max(abs(jointAcceleration), [], 'all');
    metrics.speedBoundActiveCount = nnz(speedBoundActiveLog);
    metrics.positionBoundActiveCount = nnz(positionBoundActiveLog);
    metrics.qpFailureCount = nnz(qpFallbackLog);
    metrics.constraintViolationCount = ...
        nnz(speedViolationLog | positionViolationLog);
    metrics.minimumSigma = min(sigmaMinLog);
    metrics.maxTaskResidual = max(taskResidualLog);
    metrics.recovery = recovery;

    result = struct;
    result.mode = mode;
    result.cfg = cfg;
    result.time = time;
    result.planePoint = planePoint;
    result.q = qLog;
    result.dq = dqLog;
    result.pNominal = pNominalLog;
    result.pReference = pReferenceLog;
    result.pActual = pActualLog;
    result.xAdmittance = xAdmittanceLog;
    result.vAdmittance = vAdmittanceLog;
    result.forceVector = forceVectorLog;
    result.forceMagnitude = forceMagnitudeLog;
    result.penetration = penetrationLog;
    result.nominalPositionError = nominalPositionErrorLog;
    result.referencePositionError = referencePositionErrorLog;
    result.orientationError = orientationErrorLog;
    result.sigmaMin = sigmaMinLog;
    result.taskResidual = taskResidualLog;
    result.qpExitFlag = qpExitFlagLog;
    result.qpFallback = qpFallbackLog;
    result.speedBoundActive = speedBoundActiveLog;
    result.positionBoundActive = positionBoundActiveLog;
    result.speedViolation = speedViolationLog;
    result.positionViolation = positionViolationLog;
    result.phase = phaseLog;
    result.metrics = metrics;
end
