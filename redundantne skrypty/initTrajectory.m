function trajectoryData = initTrajectory(X0, points, times)
% zainicjalizuj dane do generatora trajektorii (wymaga wywolania przed
% uzyciem funkcji trajectoryGenerator)
% wyjscie:
% trajectoryData -- tablica struktur z danymi
% wejscie:
% X0 -- punkt, w ktorym poczatkowo znajduje sie efektor robota (2*1)
% points -- tablica P punktow przez ktore ma przechodzic trajektoria (2*P)
% times -- wektor P czasow na pokonanie kazdego segmentu trajektorii
% liczba punktow to P+1 (P z points i punkt X0), wiec jest P segmentow
% trajektorii

if (length(points) ~= length(times))
    disp("initTrajectory: hola, hola, liczba punktow nie zgadza sie z liczba czasow");
    return
end

P = length(points);
trajectoryData(P, 1) = struct('Xstart', [], 'w', [], 'T', [], 'totalT', []); % wyjasnienie poszczegolnych pol w petli for ponizej
Xstart = X0;
totalT = 0;
for i = 1:P
    X = points(:, i);      % punkt docelowy i-tego segmentu trajektorii
    w = X - Xstart;         % wektor miedzy punktem poczatkowym a docelowym i-tego segmentu trajektorii
    T = times(i);           % czas na pokonanie i-tego segmentu trajektorii
    totalT = totalT + T;    % czas na pokonanie trajektorii od jej poczatku do konca i-tego segmentu
    trajectoryData(i) = struct('Xstart', Xstart, 'w', w, 'T', T, 'totalT', totalT);
    Xstart = X;             % punkt poczatkowy (i+1)-tego segmentu trajektorii
end

end