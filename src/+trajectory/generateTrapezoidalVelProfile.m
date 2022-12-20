function [q, v, a, inner_parameters] = generateTrapezoidalVelProfile( ...
        mode, q0, qk, varargin)

    availabe_modes = {'time_mode', 'vel_mode', 'sync_mode'};

    input_parser = inputParser();
    addRequired(input_parser, 'mode')
    addRequired(input_parser, 'q0')
    addRequired(input_parser, 'qk')
    addParameter(input_parser, 't0', 0)
    addParameter(input_parser, 'tk', [])
    addParameter(input_parser, 'Ta', [])
    addParameter(input_parser, 'v_m', [])
    addParameter(input_parser, 'a_m', [])
    addParameter(input_parser, 'v_max', [])
    addParameter(input_parser, 'a_max', [])

    parse(input_parser, mode, q0, qk, varargin{:})

    args = input_parser.Results;
    fields = fieldnames(args);

    for i = 1:numel(fields)

        if ~any(strcmp(fields{i}, [{'mode', 'q0', 'qk', 't0'}, varargin]))
            args = rmfield(args, fields{i});
        end

    end

    t0 = args.t0;

    if (isa(mode, 'string') || isa(mode, 'char'))

        switch mode
            case availabe_modes{1}

                if (isfield(args, 'tk') && isfield(args, 'Ta'))
                    tk = args.tk;
                    Ta = args.Ta;

                    if (Ta > (tk - t0) / 2)
                        Ta = (tk - t0) / 2;
                        warning('trajectory:generateTrapezoidalVelProfile', ...
                        'Acceleration time is greater than a half of total time, so it was reduced to the half.')
                    end

                    v_m = (qk - q0) / (tk - t0 - Ta);
                    a_m = v_m / Ta;
                else
                    error('trajectory:generateTrapezoidalVelProfile', ...
                        [mode, ' requires tk and Ta parameters.'])
                end

            case availabe_modes{2}

                if (isfield(args, 'v_m') && isfield(args, 'a_m'))
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

                if (isfield(args, 'v_m') && isfield(args, 'a_m'))
                    v_m = args.v_m(:);
                    a_m = args.a_m(:);
                    temp = (qk - q0) <= v_m .^ 2 ./ a_m;
                    Ta = zeros(numel(qk), 1);
                    T = zeros(numel(qk), 1);
                    Ta(temp) = sqrt(qk(temp) - q0(temp) ./ a_m(temp));
                    T(temp) = 2 * Ta(temp);
                    Ta(~temp) = v_m(~temp) ./ a_m(~temp);
                    Ta = max(Ta);
                    T(~temp) = (qk(~temp) - q0(~temp)) ./ v_m(~temp) + Ta;
                    tk = max(T) + t0;
                    v_m = (qk - q0) / (tk - t0 - Ta);
                    a_m = v_m / Ta;
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

    q = @(t) position_profile(t, t0, tk, Ta, q0, qk, v_m);
    v = @(t) velocity_profile(t, t0, tk, Ta, qk, v_m);
    a = @(t) acceleration_profile(t, t0, tk, Ta, qk, a_m);

    inner_parameters = struct('t0', t0, 'tk', tk, 'Ta', Ta, 'q0', q0, ...
        'qk', qk, 'v_m', v_m, 'a_m', a_m);

end

function q = position_profile(t, t0, tk, Ta, q0, qk, v_m)

    if (t <= t0)
        q = q0;
        return
    end

    if (t >= tk)
        q = qk;
        return
    end

    Ta = ones(size(qk)) .* Ta;
    v_m = ones(size(qk)) .* v_m;
    temp = (t < t0 + Ta);
    q(temp) = q0(temp) + v_m(temp) / 2 ./ Ta(temp) * (t - t0) ^ 2;
    calculated = temp;
    temp = (t < tk - Ta) & ~calculated;
    q(temp) = q0(temp) + v_m(temp) .* (t - t0 - Ta(temp) / 2);
    calculated = temp | calculated;
    q(~calculated) = qk(~calculated) - v_m(~calculated) ./ Ta(~calculated) / 2 * (tk - t) ^ 2;
end

function v = velocity_profile(t, t0, tk, Ta, qk, v_m)

    if (t <= t0 || t >= tk)
        v = zeros(size(q0));
        return
    end

    Ta = ones(size(qk)) .* Ta;
    v_m = ones(size(qk)) .* v_m;
    temp = (t < t0 + Ta);
    v(temp) = v_m(temp) ./ Ta(temp) * (t - t0);
    calculated = temp;
    temp = (t < tk - Ta) & ~calculated;
    v(temp) = v_m(temp);
    calculated = temp | calculated;
    v(~calculated) = v_m(~calculated) ./ Ta(~calculated) * (tk - t);
end

function a = acceleration_profile(t, t0, tk, Ta, qk, a_m)

    if (t < t0 || t > tk)
        a = zeros(size(qk));
        return
    end

    a_m = ones(size(qk)) .* a_m;

    if (t == t0)
        a = a_m / 2;
        return
    end

    if (t == tk)
        a = -a_m / 2;
        return
    end

    Ta = ones(size(qk)) .* Ta;
    temp = (t < t0 + Ta);
    a(temp) = a_m(temp);
    calculated = temp;
    temp = (t == (t0 + Ta));
    a(temp) = a_m(temp) / 2;
    calculated = temp | calculated;
    temp = (t < tk - Ta) & ~calculated;
    a(temp) = 0;
    calculated = temp | calculated;
    temp = (t == (tk - Ta));
    a(temp) = -a_m(temp) / 2;
    calculated = temp | calculated;
    a(~calculated) = -a_m(~calculated);
end
