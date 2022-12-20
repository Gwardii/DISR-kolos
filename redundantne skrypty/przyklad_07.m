% DiSR 
% Przyklad 6
function przyklad_07()

clear; clc;

[Q, dQdt, d2Qdt2, X, V, ErrNorm, T, H] = ikSolver();

save('wyniki.mat', 'Q', 'dQdt', 'd2Qdt2', 'X', 'V', 'ErrNorm', 'T', 'H');

end

function [Q, dQdt, d2Qdt2, X, V, ErrNorm, T, H] = ikSolver()
% wymiary manipulatora
l = [0.25 0.25 0.25 0.25];

% maksymalne i minimalne polozenie katowe
qMax = [110; 110; 110; 110] / 180 * pi;
qMin = -qMax;

% docelowa waga zadania dodatkowego (patrz: funkcja taskGain)
gammaMax = 100;                         

% konfiguracja poczatkowa
q0 = [45; -60; 60; -60] / 180 * pi;

% poczatkowe polozenie efektora manipulatora
X0 = forwardKinematics(q0, l);

% punkty trajektorii [X_1 X_2 ... X_last] (m):
points = [0.1    0.5	0.5;
          0.5    0.5	0];

% czasy na pokonanie poszczegolnych segmentow trajektorii (s):
times = [2 1 1];

% inicjalizacja struktury danych dla trajectoryGenerator:
trajectoryData = initTrajectory(X0, points, times);

% rozwiaz IK dla zadanej trajektorii:
P = length(trajectoryData);             % liczba segmentow/odcinkow trajektorii
Tend = trajectoryData(P).totalT;        % czas ruchu
dt = 0.001;                             % krok czasowy
K = 1/dt;                               % wzmocnienie do CLIK
N = ceil(Tend/dt) + 1;              	% liczba krokow
n = length(q0);                         % liczba DOF manipulatora
X = zeros(N, 2);                        % polozenia efektora
V = zeros(N, 2);                        % predkosci efektora
T = zeros(N, 1);                        % chwile czasowe
Q = zeros(N, n);                        % polozenia zlaczowe manipulatora
dQdt = zeros(N, n);                     % predkosci zlaczowe manipulatora
d2Qdt2 = zeros(N, n);                   % przyspieszenia zlaczowe manipulatora
ErrNorm = zeros(N, 1);                  % norma bledu rozwiazania IK
H = zeros(N, 1);                        % wartosc funkcji celu
q = q0;                                 % poczatkowa konfiguracja zlaczowa
t = 0;                                  % czas poczatkowy
for k = 1:N+1
    [x, v] = trajectoryGenerator(t, trajectoryData);
    X(k, :) = x';
    V(k, :) = v';
    T(k) = t;
    J = jacobian2DOF(q,l);
    err = x - forwardKinematics(q,l);
    Jp = pinv(J);
    P = eye(n) - Jp*J; % macierz projekcji
    H(k) = -goalFunction(q, l);
    gamma = taskGain(t, Tend, gammaMax);
    dphidt = - gamma * goalFunctionGradient(q, l);
    dqdt = Jp * (v + K * err) + P * dphidt;
    Q(k, :) = q';
    dQdt(k, :) = dqdt';
    if (k > 1)
       d2Qdt2(k, :) = ( dqdt' - dQdt(k-1, :) ) / dt; 
    end
    ErrNorm(k) = norm(err);
    q = q + dqdt * dt;
    t = t + dt;
end

end

function H = goalFunction(q,l)
% funkcja oblicza wartosc funkcji celu
% wyjscie:
% H -- wartosc funkcji celu
% wejscie:
% q -- biezace polozenia zlaczowe
% qMax -- maksymalne polozenia zlaczowe
% qMin -- minimalne polozenia zlaczowe

J = jacobian2DOF(q,l);
H = -sqrt( det(J * J'));

end

function dHdq = goalFunctionGradient(q, l)
% funkcja oblicza gradient funkcji celu
% wyjscie:
% dHdq -- gradient funkcji celu
% wejscie:
% q -- biezace polozenia zlaczowe
% qMax -- maksymalne polozenia zlaczowe
% qMin -- minimalne polozenia zlaczowe

dq = 0.00001;
n = length(q);
dHdq = zeros(n, 1);
H = goalFunction(q,l);
for j = 1:n
    q2 = q;
    q2(j) = q2(j) + dq;
    H2 = goalFunction(q2, l);
    dHdq(j) = (H2 - H) /dq;
end
end

function gamma = taskGain(t, Tend, gammaMax)
% funkcja generuje profil trapezowy dla wspolczynnika gamma, by predkosci
% zlaczowe byly zerowe na poczatku i koncu trajektorii
% wyjscie:
% gamma -- skalarny wspolczynnik stojacy przy null space velocity w
% rozwiazaniu IK
% wejscie:
% t -- czas biezacy
% Tend -- czas koncowy
% gammaMax -- zadana wartosc wspolczynnika gamma

t1 = 0.1 * Tend;
t2 = 0.9 * Tend;
if t < t1
    gamma = gammaMax * t/t1;
elseif t < t2
    gamma = gammaMax;
else
    gamma = gammaMax * (1 - (t-t2)/(Tend-t2));
end

end