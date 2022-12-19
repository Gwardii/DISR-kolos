classdef jointAccToTorqueTester < matlab.unittest.TestCase

    methods (Test)

        function syncModeInnerParametersTest1(testCase)
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

            jacobians = kinematics.calculatePartialArticulatedJacobians2DOFSym(q, b1, [a1; a2]);
            J_v_1 = jacobians{1, 1};
            J_v_2 = jacobians{2, 1};
            J_omega_1 = jacobians{1, 2};
            J_omega_2 = jacobians{2, 2};

            M = m1 * (J_v_1' * J_v_1) + J_omega_1' * J1 * J_omega_1 + ...
                m2 * (J_v_2' * J_v_2) + J_omega_2' * J2 * J_omega_2;
            C = 
            G = -m1 * J_v_1' * g - m2 * J_v_2' * g;

        end

    end

end
