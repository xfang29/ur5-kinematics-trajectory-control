function [pNominal, vNominal, phase] = contactNominalTrajectory(t, p0, cfg)
%CONTACTNOMINALTRAJECTORY Smooth approach, push, hold, return trajectory.

    n = cfg.contactNormal(:);
    n = n / norm(n);

    t1 = cfg.approachTime;
    t2 = t1 + cfg.pushTime;
    t3 = t2 + cfg.holdTime;
    t4 = t3 + cfg.returnTime;

    surfaceDistance = cfg.surfaceDistance;
    totalDistance = cfg.surfaceDistance + cfg.pushDepth;

    if t < t1
        [s, sdot] = quinticTimeScaling(t, cfg.approachTime);
        displacement = surfaceDistance * s;
        speed = surfaceDistance * sdot;
        phase = 'approach';

    elseif t < t2
        [s, sdot] = quinticTimeScaling(t - t1, cfg.pushTime);
        displacement = surfaceDistance + cfg.pushDepth * s;
        speed = cfg.pushDepth * sdot;
        phase = 'push';

    elseif t < t3
        displacement = totalDistance;
        speed = 0;
        phase = 'hold';

    elseif t < t4
        [s, sdot] = quinticTimeScaling(t - t3, cfg.returnTime);
        displacement = totalDistance * (1 - s);
        speed = -totalDistance * sdot;
        phase = 'return';

    else
        displacement = 0;
        speed = 0;
        phase = 'settle';
    end

    pNominal = p0 + n * displacement;
    vNominal = n * speed;
end
