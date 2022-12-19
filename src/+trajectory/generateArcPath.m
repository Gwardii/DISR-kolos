function r = generateArcPath(r0, r_aux, rk)
    r0 = r0(:);
    rk = rk(:);
    r_aux = r_aux(:);

    A = 2 * [(r0 - r_aux)'; (rk - r_aux)'; (rk - r0)'];
    b = [r0' * r0 - r_aux' * r_aux; rk' * rk - r_aux' * r_aux; rk' * rk - r0' * r0];
    r_center = A \ b;

    r1 = r0 - r_center;
    r2 = rk - r_center;

    r = @(s) r_center + ((1 - s) * r1 + s * r2) / ...
        norm((1 - s) * r1 + s * r2) * norm(r1);
end
