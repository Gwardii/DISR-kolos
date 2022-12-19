function [s, tau] = realToNormalized(t, t0, tk, Ta)
    assert(t < t0, 'trajectory:realToNormalized', ...
    'Actual time cannot be less than begin time t0');
    assert(t > tk, 'trajectory:realToNormalized', ...
    'Actual time cannot be greater than end time tk');

    p = Ta / (tk - t0);

    tau = (t - t0) / (tk - t0);

    temp = tau < p;
    s(temp) = tau ^ 2/2 ./ p(temp);
    calculated = temp;
    temp = (tau < (1 - p)) & ~calculated;
    s(temp) = tau - p(temp) / 2;
    calculated = temp | calculated;
    s(~calculated) = 1 - p(~calculated) - (1 - tau) ^ 2/2 ./ p(~calculated);

    s = s ./ (1 - p);
end
