function recovery = computeRecoveryMetrics(time, forceMagnitude, ...
                                            nominalPositionError, cfg)
%COMPUTERECOVERYMETRICS Measure release and post-release recovery times.
%
% forceReleaseDelay:
%   time from the beginning of nominal retraction until contact force stays
%   below cfg.forceThreshold for cfg.recoveryDwellTime.
%
% trackingRecoveryAfterRelease:
%   additional time after force release until both contact force and
%   nominal position error stay below their thresholds.

    retractStart = cfg.approachTime + cfg.pushTime + cfg.holdTime;

    dt = time(2) - time(1);
    dwellSamples = max(1, ceil(cfg.recoveryDwellTime / dt));

    startIndex = find(time >= retractStart, 1, 'first');

    releaseIndex = NaN;
    for k = startIndex:(numel(time) - dwellSamples + 1)
        window = k:(k + dwellSamples - 1);
        if all(forceMagnitude(window) <= cfg.forceThreshold)
            releaseIndex = k;
            break;
        end
    end

    recoveryIndex = NaN;
    if ~isnan(releaseIndex)
        for k = releaseIndex:(numel(time) - dwellSamples + 1)
            window = k:(k + dwellSamples - 1);
            condition = ...
                forceMagnitude(window) <= cfg.forceThreshold ...
                & nominalPositionError(window) <= cfg.positionTolerance;

            if all(condition)
                recoveryIndex = k;
                break;
            end
        end
    end

    if isnan(releaseIndex)
        forceReleaseDelay = NaN;
        forceReleaseTime = NaN;
    else
        forceReleaseTime = time(releaseIndex);
        forceReleaseDelay = forceReleaseTime - retractStart;
    end

    if isnan(recoveryIndex) || isnan(releaseIndex)
        trackingRecoveryAfterRelease = NaN;
        totalRecoveryFromRetract = NaN;
        recoveryTime = NaN;
    else
        recoveryTime = time(recoveryIndex);
        trackingRecoveryAfterRelease = ...
            recoveryTime - forceReleaseTime;
        totalRecoveryFromRetract = recoveryTime - retractStart;
    end

    recovery = struct;
    recovery.retractStart = retractStart;
    recovery.forceReleaseTime = forceReleaseTime;
    recovery.forceReleaseDelay = forceReleaseDelay;
    recovery.recoveryTime = recoveryTime;
    recovery.trackingRecoveryAfterRelease = ...
        trackingRecoveryAfterRelease;
    recovery.totalRecoveryFromRetract = totalRecoveryFromRetract;
end
