% DiSR 
% Przyklad 5
function przyklad_05()

clear; clc;

[Q, dQdt, d2Qdt2, X, V, ErrNorm, T] = ikSolver();

save('wyniki.mat', 'Q', 'dQdt', 'd2Qdt2', 'X', 'V', 'ErrNorm', 'T');

end

function [Q, dQdt, d2Qdt2, X, V, ErrNorm, T] = ikSolver()
% wymiary manipulatora
l = [0.25 0.25 0.25 0.25];

% konfiguracja poczatkowa
q0 = [45; -90; 90; -90] / 180 * pi;

% poczatkowe polozenie efektora manipulatora
X0 = forwardKinematics(q0, l);

% punkty trajektorii [X_1 X_2 ... X_last] (m):
points = [0.8	0.8 0.5 0.5;
          0     0.5	0.5 0];

% czasy na pokonanie poszczegolnych segmentow trajektorii (s):
times = [0.5 1 1 1];

% inicjalizacja struktury danych dla trajectoryGenerator:
trajectoryData = initTrajectory(X0, points, times);

dqdt_lim = [50; 120; 200; 200] /180 * pi;
dqdt_lim_min = min(dqdt_lim);

W = diag((dqdt_lim_min./dqdt_lim).^2);

% rozwiaz IK dla zadanej trajektorii:
P = length(trajectoryData);             % liczba segmentow/odcinkow trajektorii
Tend = trajectoryData(P).totalT;        % czas ruchu
dt = 0.01;                              % krok czasowy
N = ceil(Tend/dt) + 1;              	% liczba krokow
n = length(q0);                         % liczba DOF manipulatora
X = zeros(N, 2);                        % polozenia efektora
V = zeros(N, 2);                        % predkosci efektora
T = zeros(N, 1);                        % chwile czasowe
Q = zeros(N, n);                        % polozenia zlaczowe manipulatora
dQdt = zeros(N, n);                     % predkosci zlaczowe manipulatora
d2Qdt2 = zeros(N, n);                   % przyspieszenia zlaczowe manipulatora
ErrNorm = zeros(N, 1);                  % norma bledu rozwiazania IK
q = q0;                                 % poczatkowa konfiguracja zlaczowa
t = 0;                                  % czas poczatkowy
K = eye(2)*0.1/dt;
for k = 1:N+1
    [x, v] = trajectoryGenerator(t, trajectoryData);
    X(k, :) = x';
    V(k, :) = v';
    T(k) = t;
    J = jacobian2DOF(q, l);
    err = x - forwardKinematics(q,l);
    dqdt = W^(-1/2) * pinv(J *  W^(-1/2) )*(v + K*err);
    %dqdt = pinv(J)*v;
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