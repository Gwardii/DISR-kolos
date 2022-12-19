function [v, x, a] = generateTrapezoidalVelProfile(mode, q0, qk, varargin)

    availabe_modes = {'time_mode', 'vel_mode', 'sync_mode'};

    input_parser = inputParser();
    addRequired(input_parser, 'mode')
    addRequired(input_parser, 'q0')
    addRequired(input_parser, 'qk')
    addParameter(input_parser, 't0')
    addParameter(input_parser, 'tk')
    addParameter(input_parser, 'Ta')
    addParameter(input_parser, 'v_m')
    addParameter(input_parser, 'a_m')
    addParameter(input_parser, 'v_max')
    addParameter(input_parser, 'a_max')

    parse(input_parser, mode, q0, qk, varargin{:})

    args = input_parser.Results;
    fields = fieldnames(args);

    for i = 1:numel(fields)

        if ~any(strcmp(fields{i}, [{'mode', 'q0', 'qk'}, varargin]))
            args = rmfield(args, fields{i});
        end

    end

    if (isfield(args, 't0'))
        t0 = args.('t0');
    else
        t0 = 0;
    end

    if (isa(mode, 'string') | isa(mode, 'char'))

        switch mode
            case availabe_modes{1}
                mode_id = 0;

                if (isfield(args, 'tk') & isfield(args, 'Ta'))
                    tk = args.tk;
                    Ta = args.Ta;

                    if (Ta > (tk - t0) / 2)
                        Ta = (tk - t0) / 2;
                        warning('trajectory:generateTrapezoidalVelProfile', ...
                            'Acceleration time is greater than a half of ...
                            total time, so it was reduced to the half.')
                    end

                    v_m = (qk - q0) / tk - t0 - Ta;
                    a_m = v_m / Ta;
                else
                    error('trajectory:generateTrapezoidalVelProfile', ...
                        [mode, ' requires tk and Ta parameters.'])
                end

            case availabe_modes{2}
                mode_id = 1;

                if (isfield(args, 'v_m') & isfield(args, 'a_m'))
                    v_m = args.v_m(:);
                    a_m = args.a_m(:);
                    temp = (qk - q0) <= v_m .^ 2 ./ a_m;
                    Ta = zeros(1, numel(qk));
                    Ta(temp) = sqrt(qk(temp) - q0(temp) / a_m);
                    Ta(~temp) = v_m / a_m;
                    tk = (qk - q0) ./ v_m + T_a + t0;
                else
                    error('trajectory:generateTrapezoidalVelProfile', ...
                        [mode, ' requires v_m and a_m parameters.'])
                end

            case availabe_modes{3}
                mode_id = 2;

                if (isfield(args, 'v_m') && isfield(args, 'a_m'))
                    v_m = args.v_m(:);
                    a_m = args.a_m(:);
                    temp = (qk - q0) <= v_m .^ 2 ./ a_m;
                    Ta = zeros(numel(qk), 1);
                    T = zeros(numel(qk), 1);
                    Ta(temp) = sqrt(qk(temp) - q0(temp) / a_m);
                    T(temp) = 2 * Ta(temp);
                    Ta(~temp) = v_m / a_m;
                    Ta = max(Ta);
                    T(~temp) = (qk - q0) ./ v_m + Ta;
                    tk = max(T) + t0;
                else
                    error('trajectory:generateTrapezoidalVelProfile', ...
                        [mode, ' requires v_m and a_m parameters.'])
                end

            otherwise
                error('trajectory:generateTrapezoidalVelProfile', ...
                    '%s is not supported. Availabe modes: %s.', ...
                    mode, join(availabe_modes, ', '))
        end

    else
        error('trajectory:generateTrapezoidalVelProfile', ...
        'Wrong input type. Mode should be a char array or a string.')
    end

end

function q = velocity_profile(t, t0, tk, Ta, q0, qk, v_m)
    t0 = ones(size(tk)) * t0;
    temp = t <= t0;
    q(temp) = q0(temp);
    calculated = temp;
    temp = (t < tk - Ta) & ~calculated;
    q(temp) = q0(temp) + v_m(temp) .* (t - t0 - Ta(temp) / 2);
    calculated = temp & calculated;
    temp = (t < tk) & ~calculated;
    q(temp) = qk(temp) - v_m(temp) ./ Ta(temp) / 2;
    calculated = temp & calculated;
    q(~calculated) = qk(~calculated);
end
