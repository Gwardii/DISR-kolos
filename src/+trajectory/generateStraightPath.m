function r = generateStraightPath(r0, rk)
    r = @(s) r0 + (rk - r0) * s;
end
