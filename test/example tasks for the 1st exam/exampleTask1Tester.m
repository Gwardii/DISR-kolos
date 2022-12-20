classdef exampleTask1Tester < matlab.unittest.TestCase

    methods (Test)

        function task1(testCase)
            % First example task
            t0 = 0;
            q0 = [0; 2]; % [rad]
            qk = [4; 5]; % [rad]
            v_max = [3; 2]; % [rad/s]
            a_max = [1; 2]; %[rad/s^2]
            [~, ~, ~, inner_parameters] = trajectory.generateTrapezoidalVelProfile( ...
                'sync_mode', q0, qk, 'v_m', v_max, 'a_m', a_max, 't0', t0);
            verifyEqual(testCase, inner_parameters.Ta, 2, ...
            'Wrong acceleration time in a sync mode.')
            verifyEqual(testCase, inner_parameters.tk, 4, ...
            'Wrong end time in a sync mode.')
            verifyEqual(testCase, inner_parameters.v_m, [2; 1.5], ...
            'Wrong velocities in a sync mode.')
            verifyEqual(testCase, inner_parameters.a_m, [1; 0.75], ...
            'Wrong acceleration in a sync mode.')
        end

    end

end
