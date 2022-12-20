classdef exampleTask3Tester < matlab.unittest.TestCase

    methods (Test)

        function task3(testCase)
            a1 = 0.5;
            b1 = 1;
            a2 = 0.2;
            m1 = 20;
            m2 = 5;
            J1 = diag([0.2, 2, 2]);
            J2 = diag([0.01, 0.1, 0.1]);
            g = [0; -10; 0];
            q_ = [0; pi / 2];
            dq = [1; 2];
            ddq = [-20/3; 10];

            syms q [2, 1]

            jacobians = kinematics.calculatePartialArticulatedJacobians2DOFSym(q, b1, [a1; a2]);
            J_v_1 = jacobians{1, 1};
            J_v_2 = jacobians{2, 1};
            J_omega_1 = jacobians{1, 2};
            J_omega_2 = jacobians{2, 2};

            M = m1 * (J_v_1' * J_v_1) + J_omega_1' * J1 * J_omega_1 + ...
                m2 * (J_v_2' * J_v_2) + J_omega_2' * J2 * J_omega_2;
            C = dynamics.calculateChristoffelMatrixSym(M, dq);
            G = -m1 * J_v_1' * g - m2 * J_v_2' * g;
            q1 = q_(1);
            q2 = q_(2);
            M = double(subs(M));
            C = double(subs(C));
            G = double(subs(G));
            tau = M * ddq + C * dq + G;
            testCase.verifyLessThan(abs(tau - [63; 3]), 1e-13)

        end

        function task3recursion(testCase)
            a1 = 0.5;
            b1 = 1;
            a2 = 0.2;
            m1 = 20;
            m2 = 5;
            J1 = diag([0.2, 2, 2]);
            J2 = diag([0.01, 0.1, 0.1]);
            g = [0; -10; 0];
            q = [0; pi / 2];
            dq = [1; 2];
            ddq = [-20/3; 10];

            R_z = @(phi) [utils.omega(phi), zeros(2, 1); 0, 0, 1];

            w_1 = [0; 0; 1];
            r_01 = zeros(3, 1);
            dr_01 = zeros(3, 1);
            ddr_01 = zeros(3, 1);
            R_10 = R_z(q1);
            omega_1 = R_10 * w * dq(1);
            eps_1 = R_10 * w * ddq(1);

            s_C1 = [a1; 0; 0];
            r_C1 = r_01 + R_10 * s_C1;
            dr_C1 = dr_01 + utils.attachedMatrix(omega_1) * R_10 * s_C1;
            ddr_C1 = ddr_01 + utils.attachedMatrix(eps_1) * R_10 * s_C1 + ...
                utils.attachedMatrix(omega) * utils.attachedMatrix(omega) * R_10 * s_C1;

        end

    end

end
