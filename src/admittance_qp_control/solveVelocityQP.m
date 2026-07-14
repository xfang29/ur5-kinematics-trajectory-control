function [dq, info] = solveVelocityQP(J, Vcmd, q, dqPrevious, cfg)
%SOLVEVELOCITYQP Constrained inverse-velocity QP.
%
% Solves
%
%   min_dq  1/2 ||J*dq - Vcmd||_W^2
%         + alpha/2 ||dq||_2^2
%         + beta/2  ||dq - dqPrevious||_2^2
%
% subject to joint-speed and one-step joint-position bounds.

    arguments
        J double
        Vcmd double
        q double
        dqPrevious double
        cfg struct
    end

    persistent qpOptions

    if isempty(qpOptions)
        if exist('quadprog', 'file') ~= 2
            error('solveVelocityQP:MissingQuadprog', ...
                ['quadprog was not found. This experiment requires ', ...
                 'MATLAB Optimization Toolbox.']);
        end

        qpOptions = optimoptions('quadprog', ...
            'Algorithm', 'active-set', ...
            'Display', 'off');
    end

    q = q(:);
    dqPrevious = dqPrevious(:);
    Vcmd = Vcmd(:);

    W = cfg.taskWeight;
    alpha = cfg.velocityRegularization;
    beta = cfg.smoothnessWeight;

    H = J.' * W * J + (alpha + beta) * eye(size(J,2));
    H = 0.5 * (H + H.');
    f = -J.' * W * Vcmd - beta * dqPrevious;

    speedLower = -cfg.jointSpeedMax(:);
    speedUpper =  cfg.jointSpeedMax(:);

    positionLower = ...
        (cfg.jointMin(:) + cfg.jointMargin(:) - q) / cfg.dt;
    positionUpper = ...
        (cfg.jointMax(:) - cfg.jointMargin(:) - q) / cfg.dt;

    lowerBound = max(speedLower, positionLower);
    upperBound = min(speedUpper, positionUpper);

    if any(lowerBound > upperBound)
        error('solveVelocityQP:InconsistentBounds', ...
            ['The current joint state is outside the configured safety ', ...
             'envelope or the bounds are inconsistent.']);
    end

    x0 = min(max(dqPrevious, lowerBound), upperBound);

    [dqCandidate, objectiveValue, exitFlag, output] = quadprog( ...
        H, f, [], [], [], [], lowerBound, upperBound, x0, qpOptions);

    usedFallback = exitFlag <= 0 || isempty(dqCandidate) ...
                   || any(~isfinite(dqCandidate));

    if usedFallback
        % Preserve safety if the numerical solver fails.
        dq = x0;
    else
        dq = dqCandidate;
    end

    tolerance = 1e-6;

    speedBoundActive = any( ...
        abs(dq - speedLower) <= tolerance | ...
        abs(dq - speedUpper) <= tolerance);

    positionBoundActive = any( ...
        abs(dq - positionLower) <= tolerance | ...
        abs(dq - positionUpper) <= tolerance);

    qNext = q + dq * cfg.dt;

    speedViolation = any(dq < speedLower - tolerance) ...
                  || any(dq > speedUpper + tolerance);

    positionViolation = ...
        any(qNext < cfg.jointMin(:) + cfg.jointMargin(:) - tolerance) ...
        || any(qNext > cfg.jointMax(:) - cfg.jointMargin(:) + tolerance);

    info = struct;
    info.exitFlag = exitFlag;
    info.output = output;
    info.objectiveValue = objectiveValue;
    info.usedFallback = usedFallback;
    info.lowerBound = lowerBound;
    info.upperBound = upperBound;
    info.positionLower = positionLower;
    info.positionUpper = positionUpper;
    info.speedBoundActive = speedBoundActive;
    info.positionBoundActive = positionBoundActive;
    info.speedViolation = speedViolation;
    info.positionViolation = positionViolation;
end
