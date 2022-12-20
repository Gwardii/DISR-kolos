function J = jacobian2DOF(q, l)
% jakobian manipulatora plaskiego
% wyjscie:
% J -- jakobian (2*n)
% wejscie:
% q -- wspolrzedne zlaczowe (n*1)
% l -- dlugosci czlonow (n*1)

    if (length(q) ~= length(l))
        disp("jacobian: q i l maja rozna dlugosc");
        return
    end

    n = length(q);
    J = zeros(2, n);

    for j = 1:n
        for i = j:n
            J(1, j) = J(1, j) - l(i) * sin( sum( q(1:i) ) );
            J(2, j) = J(2, j) + l(i) * cos( sum( q(1:i) ) );
        end
    end

end