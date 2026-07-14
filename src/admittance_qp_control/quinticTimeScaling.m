function [s, sdot] = quinticTimeScaling(t, duration)
%QUINTICTIMESCALING Fifth-order time scaling from 0 to 1.
%
% s(0)=0, s(T)=1, and both endpoint velocities and accelerations are zero.

    arguments
        t (1,1) double
        duration (1,1) double {mustBePositive}
    end

    tau = min(1, max(0, t / duration));

    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    sdot = (30*tau^2 - 60*tau^3 + 30*tau^4) / duration;
end
