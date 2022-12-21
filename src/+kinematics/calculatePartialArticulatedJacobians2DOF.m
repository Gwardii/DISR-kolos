function jacobians = calculatePartialArticulatedJacobians2DOF(q, l, c)
    n = length(q);
    R = @(q_) [utils.omega(q_), zeros(2, 1); 0, 0, 1];
    u_tilde = utils.attachedMatrix([0; 0; 1]);

    J_v = zeros(3, n);
    J_omega = zeros(3, n);

    d = zeros(3, n);

    for i = 1:n

        d(:, i) = R(sum(q(1:i))) * [c(i); 0; 0];
        J_v(:, i) = u_tilde * d(:, i);

        for j = (i - 1):-1:1
            d(:, j) = d(:, j + 1) + R(q(j)) * [l(j); 0; 0];
            J_v(:, j) = u_tilde * d(:, j);
        end

        jacobians{i, 1} = J_v;
        J_omega(3, 1:i) = 1;
        jacobians{i, 2} = J_omega;
    end

    if length(l) > n
        d(:, n) = R(sum(q(1:n))) * [l(n); 0; 0];
        J_v(:, n) = u_tilde * d(:, n);

        for j = (n - 1):-1:1
            d(:, j) = d(:, j + 1) + R(q(j)) * [l(j); 0; 0];
            J_v(:, j) = u_tilde * d(:, j);
        end

        jacobians{n + 1, 1} = J_v;
        jacobians{n + 1, 2} = J_omega;

    end
