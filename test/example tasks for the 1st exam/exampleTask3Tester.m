classdef exampleTask3Tester < matlab.unittest.TestCase

    methods (Test)

        function task3(testCase)
            R_z = @(phi)[utils.omega(phi),zeros(2,1);0,0,1];
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

            syms q [2, 1] real

            jacobians = kinematics.calculatePartialArticulatedJacobians2DOFSym(q, b1, [a1; a2]);
            J_v_1 = jacobians{1, 1};
            J_v_2 = jacobians{2, 1};
            J_omega_1 = jacobians{1, 2};
            J_omega_2 = jacobians{2, 2};

            R_10 = R_z(q1);
            J1_0 = R_10*J1*R_10';
            R_20 = R_z(q1+q2);
            J2_0 = R_20*J2*R_20';

            M = m1 * (J_v_1' * J_v_1) + J_omega_1' * J1_0 * J_omega_1 + ...
                m2 * (J_v_2' * J_v_2) + J_omega_2' * J2_0 * J_omega_2;
            C = dynamics.calculateChristoffelMatrixSym(M, dq);
            G = -m1 * J_v_1' * g - m2 * J_v_2' * g;
            q1 = q_(1);
            q2 = q_(2);
            M = double(subs(M));
            C = double(subs(C));
            G = double(subs(G));
            tau = M * ddq + C * dq + G;
            testCase.verifyLessThan(abs(tau - [63; 2]), 1e-13)

        end

        function task3recursion(testCase)
            a1 = 0.5;
            b1 = 1;
            a2 = 0.2;
            m1 = 20;
            m2 = 5;
            J_C1 = diag([0.2, 2, 2]);
            J_C2 = diag([0.01, 0.1, 0.1]);
            g = [0; -10; 0];
            q = [0; pi / 2];
            dq = [1; 2];
            ddq = [-20/3; 10];

            R_z = @(phi) [utils.omega(phi), zeros(2, 1); 0, 0, 1];

            w = [0; 0; 1];

            r_01 = zeros(3, 1);
            dr_01 = zeros(3, 1);
            ddr_01 = zeros(3, 1);
            R_10 = R_z(q(1));
            omega_01 = w * dq(1);
            eps_01 = w * ddq(1);

            s_C1 = [a1; 0; 0];
            r_C1 = r_01 + R_10 * s_C1;
            dr_C1 = dr_01 + utils.attachedMatrix(omega_01) * R_10 * s_C1;
            ddr_C1 = ddr_01 + utils.attachedMatrix(eps_01) * R_10 * s_C1 + ...
                utils.attachedMatrix(omega_01) * utils.attachedMatrix(omega_01) * R_10 * s_C1;

            u_1 = R_10 * w;
            J_C1_0 = R_10 * J_C1 * R_10';

            r_12 = [b1; 0; 0];
            dr_12 = zeros(3, 1);
            ddr_12 = zeros(3, 1);
            R_21 = R_z(q(2));
            omega_12 = w * dq(2);
            eps_12 = w * ddq(2);

            s_C2 = [a2; 0; 0];

            r_02 = r_01 + R_10 * r_12;
            dr_02 = dr_01 + utils.attachedMatrix(omega_01) * R_10 * r_12 + R_10 * dr_12;
            ddr_02 = ddr_01 + utils.attachedMatrix(eps_01) * R_10 * r_12 + ...
                utils.attachedMatrix(omega_01) * utils.attachedMatrix(omega_01) * R_10 * r_12 + ...
                2 * utils.attachedMatrix(omega_01) * R_10 * dr_12 + R_10 * ddr_12;

            R_20 = R_10 * R_21;
            omega_02 = omega_01 + R_10 * omega_12;
            eps_02 = eps_01 + utils.attachedMatrix(omega_01) * R_10 * omega_12 + R_10 * eps_12;

            r_C2 = r_02 + R_20 * s_C2;
            dr_C2 = dr_02 + utils.attachedMatrix(omega_02) * R_20 * s_C2;
            ddr_C2 = ddr_02 + utils.attachedMatrix(eps_02) * R_20 * s_C2 + ...
                utils.attachedMatrix(omega_02) * utils.attachedMatrix(omega_02) * R_20 * s_C2;

            u_2 = R_20 * w;
            J_C2_0 = R_20 * J_C2 * R_20';

            F_12 = m2*(ddr_C2 - g);
            T_12 = J_C2_0 * eps_02 + utils.attachedMatrix(omega_02) * J_C2_0 * omega_02 - utils.attachedMatrix(r_02 - r_C2) * F_12;
            F_01 = m1*(ddr_C1 - g) + F_12;
            T_01 = J_C1_0 * eps_01 + utils.attachedMatrix(omega_01) * J_C1_0 * omega_01 - ...
                utils.attachedMatrix(r_01 - r_C1) * F_01 + utils.attachedMatrix(r_02 - r_C1) * F_12 + T_12;
            
            testCase.verifyLessThan(norm([T_01(3);T_12(3)] - [63; 2]), 1e-13)
        end

    end

end
