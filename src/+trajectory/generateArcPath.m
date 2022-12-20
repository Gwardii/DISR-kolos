function r = generateArcPath(r0, r_aux, rk)
    r0 = r0(:);
    rk = rk(:);
    r_aux = r_aux(:);

    A = 2 * [(r0 - r_aux)'; (rk - r_aux)'; (rk - r0)'];
    b = [r0' * r0 - r_aux' * r_aux; rk' * rk - r_aux' * r_aux; rk' * rk - r0' * r0];
    r_center = A \ b;

    r1 = r0 - r_center;
    r2 = rk - r_center;

    w = cross(r1, r2);
    temp = norm(w);
    w = w / temp;
    temp = temp / norm(r1) / norm(r2);
    angle = acos(temp);
    R = @(phi) utils.R(phi, w);

    r = @(s) r_center + R(angle * s) * r1;
end
