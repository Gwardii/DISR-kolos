% DiSR 
% Przyklad 9
function przyklad_09()

clear; clc;

[Q, dQdt, d2Qdt2, X, V, ErrNorm, T, S, S_mod] = ikSolver();

save('wyniki.mat', 'Q', 'dQdt', 'd2Qdt2', 'X', 'V', 'ErrNorm', 'T', 'S', 'S_mod');

end

function [Q, dQdt, d2Qdt2, X, V, ErrNorm, T, S, S_mod] = ikSolver()
% wymiary manipulatora
l = [0.25 0.25 0.25 0.25];

% parametry dla DLS i SVF:
kappa_max = 0.1;
epsilon = 0.2;
sigma0 = 0.6;
nu = 0.1;

% konfiguracja poczatkowa
q0 = [45; -90; 90; -90] / 180 * pi;

% poczatkowe polozenie efektora manipulatora
X0 = forwardKinematics(q0, l);

% punkty trajektorii [X_1 X_2 ... X_last] (m):
points = [0.8	0.8 0.5253 0.5;
          0     0.5	0.8509 0];

% czasy na pokonanie poszczegolnych segmentow trajektorii (s):
times = [0.5 1 1 1];

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
S = zeros(N, 2);                        % wartosci szczegolne
S_mod = zeros(N, 2);                    % zmodyfikowane wartosci szczegolne
q = q0;                                 % poczatkowa konfiguracja zlaczowa
t = 0;                                  % czas poczatkowy
for k = 1:N+1
    [x, v] = trajectoryGenerator(t, trajectoryData);
    X(k, :) = x';
    V(k, :) = v';
    T(k) = t;
    J = jacobian2DOF(q, l);
    err = x - forwardKinematics(q, l);
    S(k, :) = svd(J)';
    [Jp, s_mod] = dampedPseudoinverse(J, kappa_max, epsilon);
%     [Jp, s_mod] = SVFPseudoinverse(J, sigma0, epsilon, nu);
    S_mod(k, :) = s_mod;
    dqdt = Jp * (v + K * err);
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

function [Jp, s_mod] = dampedPseudoinverse(J, kappa_max, epsilon)
% wyjscie:
% Jp -- pseudoodwrotnosc jakobianu
% s_mod -- zmodyfikowane wartosci szczegolne
% wejscie:
% J -- jakobian manipulatora
% kappa_max -- maksymalny wspolczynnik kappa
% epsilon -- wartosc progowa najmniejszej wartosci szczegolnej, dla ktorej
% aktywowany jest zmienny wspolczynnik kappa



end

function [Jp, s_mod] = SVFPseudoinverse(J, sigma0, epsilon, nu)
% wyjscie:
% Jp -- pseudoodwrotnosc jakobianu
% s_mod -- zmodyfikowane wartosci szczegolne
% wejscie:
% J -- jakobian manipulatora
% sigma0 -- maksymalny wspolczynnik kappa
% epsilon -- wartosc progowa najmniejszej wartosci szczegolnej, dla ktorej
% aktywowana jest funkcja filtrowania wartosci szczegolnych
% nu -- wspolczynnik funkcji filtrowania



end