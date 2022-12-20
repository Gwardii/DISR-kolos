classdef exampleTask4Tester < matlab.unittest.TestCase

    methods (Test)

        function task4(testCase)
            h = 1;
            m1 = 20;
            m2 = 10;
            J1 = diag([0.2, 2, 2]);
            J2 = diag([0.1, 1, 1]);
            F = [100; 200; 0];
            g = [0; -10; 0];
            q_ = [pi / 4; 1];
            dq_ = [1; 2];
            tau = [10; -100];

            syms q dq ddq [2, 1]
            R_z = @(phi)[utils.omega(phi), zeros(2, 1); 0, 0, 1];
            r_2 = R_z(q1) * [q2; 0; 0];
            r_tcp = R_z(q1) * [q2 + h; 0; 0];
            dr_2 = jacobian(r_2) * dq;
            E_k = 0.5 * (m2 * dr_2' * dr_2 + J1(3, 3) * dq1 ^ 2 + J2(3, 3) * dq1 ^ 2);
            E_p = -g' * r_2 * m2;
            L_dq = jacobian(E_k, dq);
            L_q = jacobian(E_k - E_p, q);
            dL_dq = [dq; ddq]' * jacobian(L_dq, [q; dq])';
            J_v_tcp = jacobian(r_tcp, q);
            Q = J_v_tcp' * F;

            q1 = q_(1);
            q2 = q_(2);
            dq1 = dq_(1);
            dq2 = dq_(2);

            dL_dq = subs(dL_dq);
            L_q = subs(L_q);
            Q = subs(Q);
            solution = solve(dL_dq' + L_q' == Q);

            ddq = double([solution.ddq1; solution.ddq2]);

            testCase.verifyLessThan(norm(ddq - ...
                [13.240925719689558; 27.284271247461902]), 1e-15)

        end

    end

end
