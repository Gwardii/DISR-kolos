function [Jp, P] = pseudoInverseMatrix(J, varargin)
    availabe_tasks = {'none'};

    input_parser = inputParser();
    addParameter(input_parser, 'W', eye(size(J, 2)))
    addParameter(input_parser, 'extra_task', availabe_tasks{1})

    parse(input_parser, varargin{:})

    args = input_parser.Results;
    fields = fieldnames(args);

    for i = 1:numel(fields)

        if ~any(strcmp(fields{i}, [{'W', 'extra_task'}, varargin]))
            args = rmfield(args, fields{i});
        end

    end

    Jp = W ^ (-0.5) * pinv(J * W ^ (-0.5));
    P = eye(size(J, 2) - Jp * J);

end
