function x = forwardKinematics(q, l)
% kinematyka prosta manipulatora plaskiego
% wyjscie:
% x -- polozenie koncowki na plaszczyznie (2DOF)
% wejscie:
% q -- wspolrzedne zlaczowe
% l -- dlugosci czlonow

    if (length(q) ~= length(l))
        disp("forwardKin: q i l maja rozna dlugosc");
        return
    end

    n = length(q);
    x = zeros(2,1);

    for j = 1:n
       x(1) = x(1) + l(j) * cos( sum( q(1:j) ) );
       x(2) = x(2) + l(j) * sin( sum( q(1:j) ) );
    end
    
end