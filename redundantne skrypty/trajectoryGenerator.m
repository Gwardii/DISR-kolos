function [x, v] = trajectoryGenerator(t, trajectoryData)
% generator trajektorii 2D
% wyjscie:
% x -- polozenie koncowki na plaszczyznie
% v -- predkosc koncowki na plaszczyznie
% wejscie:
% t -- czas
% trajectoryData -- dane trajektorii (patrz: funkcja initTrajectory)
    
    N = length(trajectoryData); % liczba segmentow trajektorii
    % sprawdz jaki powinien byc teraz numer segmentu trajektorii
    i = 1;
    while (t > trajectoryData(i).totalT)
       i = i + 1;
       if (i > N)
           i = N;
           break;
       end
    end
    
    if (i > 1)
        t = t - trajectoryData(i - 1).totalT;   % na potrzeby biezacego segmentu trajektorii liczymy czas od 0 do T
    end
    
    [u, dudt] = timeLaw(t, trajectoryData(i).T);
    x = trajectoryData(i).Xstart + trajectoryData(i).w * u;
    v = trajectoryData(i).w * dudt;
end

function [u, dudt] = timeLaw(t, T)
% funkcja opisujaca przemieszczenie i predkosc wzdluz sciezki
% wyjscie:
% u -- przemieszczenie wzdluz sciezki (znormalizowane do dlugosci sciezki: u nalezy do <0, 1>)
% dudt -- predkosc wzdluz sciezki (rowniez znormalizowane)
% wejscie:
% t -- lokalny czas (od punktu startowego do docelowego)
% T -- czas trwania biezacego segmentu trajektorii
    
    if t < 0
        u = 0;
        dudt = 0;
    elseif t <= T
        u = 6 / T^5 * t^5 - 15 / T^4 * t^4 + 10 / T^3 * t^3;
        dudt = 30 / T^5 * t^4 - 60 / T^4 * t^3 + 30 / T^3 * t^2;
    else
        u = 1;
        dudt = 0;
    end
end