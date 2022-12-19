classdef inverseKinematicsTester < matlab.unittest.TestCase

    methods (Test)

        function syncModeInnerParametersTest1(testCase)
            t0 = 0;
            tk = 10;
            Ta = 2;
            ra = [0.8; 0.6];
            rb = [1.6; 0.2];
            l1 = 1;
            l2 = 0.8;

            t = 3;

            s = trajectory.realToNormalized(t, t0, tk, Ta);
            r = trajectory.generateStraightPath(ra, rb);
            r = r(s);

            pom = (r' * r - l1 ^ 2 - l2 ^ 2) / (2 * l1 * l2);
            q(2, :) = acos(pom);
            q(1, :) = atan2(r(2) * (l1 + l2 * cos(q(2))) - r(1) * l2 * sin(q(2)), ...
                r(1) * (l1 + l2 * cos(q(2))) + r(2) * l2 * sin(q(2)));
            testCase.verifyLessThan(norm(q - [-0.303326385290135;1.81702693523074]),1e-14)

        end

    end

end
