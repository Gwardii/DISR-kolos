% DiSR 
% Przyklad 8
function przyklad_08()

clear; clc;

[Q, dQdt, d2Qdt2, X, V, ErrNorm, T, D] = ikSolver();

save('wyniki.mat', 'Q', 'dQdt', 'd2Qdt2', 'X', 'V', 'ErrNorm', 'T', 'D');

end

function [Q, dQdt, d2Qdt2, X, V, ErrNorm, T, D] = ikSolver()
% wymiary manipulatora
l = [0.25 0.25 0.25 0.25];

% konfiguracja poczatkowa
q0 = [-45; 90; -90; 90] / 180 * pi;

% poczatkowe polozenie efektora manipulatora
X0 = forwardKinematics(q0, l);

% punkty trajektorii [X_1 X_2 ... X_last] (m):
points = [0.8	0.8 0.5 0.5;
          0     0.5	0.5 0];

% czasy na pokonanie poszczegolnych segmentow trajektorii (s):
times = [0.5 1 1 1];

% inicjalizacja struktury danych dla trajectoryGenerator:
trajectoryData = initTrajectory(X0, points, times);

% dane do zadania omijania przeszkody:
o = [0.25; 0.25];       % polozenie przeszkody
d_ug = 0.2;            % odleglosc od przeszkody, dla ktorej zadanie omijania ma maksymalna wage
d_soi = 0.3;            % odleglosc od przeszkody, dla ktorej nastepuje aktywacja zadania omijania
gamma_s_max = 5;        % maksymalna waga zadania omijania
vO_nom = 1;             % predkosc oddalania sie od przeszkody
obstacleData = struct('o', o, 'd_ug', d_ug, 'd_soi', d_soi, 'gamma_s_max', gamma_s_max, 'vO_nom', vO_nom);

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
D = zeros(N, 1);                        % odleglosc od przeszkody
q = q0;                                 % poczatkowa konfiguracja zlaczowa
t = 0;                                  % czas poczatkowy
for k = 1:N+1
    [x, v] = trajectoryGenerator(t, trajectoryData);
    X(k, :) = x';
    V(k, :) = v';
    T(k) = t;
    J = jacobian2DOF(q, l);
    Jp = pinv(J);
    err = x - forwardKinematics(q, l);
    [JdO, o_dist, gamma_s, vO] = obstacleAvoidance(q, l, obstacleData);
    D(k) = o_dist;
    dqdt = Jp * (v + K * err) + gamma_s * pinv( JdO * (eye(n,n) - Jp * J) ) * (vO - JdO * Jp * v);
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

function [JdO, o_dist, gamma_s, vO] = obstacleAvoidance(q, l, obstacleData)
% oblicza niezbedne zmienne do zadania omijania przeszkody
% wyjscie:
% JdO -- jednowymiarowy jakobian zadania omijania przeszkody
% o_dist -- odleglosc od przeszkody
% gamma_s -- waga zadania omijania przeszkody
% vO -- predkosc ucieczki od przeszkody
% wejscie:
% q -- wspolrzedne zlaczowe
% l -- dlugosci czlonow
% obstacleData -- struktura z danymi do zadania omijania przeszkody

rO = obstacleForwardKinematics(q, l);
o = obstacleData.o;
dO = rO - o;
o_dist = norm(dO);
miO = dO / o_dist;

JO = obstacleJacobian(q, l);
JdO = miO' * JO;

d_ug = obstacleData.d_ug;
d_soi = obstacleData.d_soi;
if o_dist <= d_ug
    gamma_s = 1;
    gamma_v = ( d_ug / o_dist )^2 - 1;
elseif o_dist < d_soi
    gamma_s = 0.5 * ( 1 - cos( pi * (o_dist - d_soi) / (d_ug - d_soi) ) );
    gamma_v = 0;
else
    gamma_s = 0;
    gamma_v = 0;
end
gamma_s_max = obstacleData.gamma_s_max;
gamma_s = gamma_s * gamma_s_max;

vO_nom = obstacleData.vO_nom;
vO = gamma_v * vO_nom;

end

function rO = obstacleForwardKinematics(q, l)
% oblicza polozenie punktu omijajacego na robocie
% wyjscie:
% rO -- polozenie punktu omijajacego
% wejscie:
% q -- wspolrzedne zlaczowe
% l -- dlugosci czlonow

    if (length(q) ~= length(l))
        disp("obstacleForwardKin: q i l maja rozna dlugosc");
        return
    end

    rO = zeros(2,1);
    % punkt omijajacy przeszkody jest na lokciu robota (na koncu drugiego
    % czlonu)
    for j = 1:2
       rO(1) = rO(1) + l(j) * cos( sum( q(1:j) ) );
       rO(2) = rO(2) + l(j) * sin( sum( q(1:j) ) );
    end
end

function JO = obstacleJacobian(q, l)
% jakobian dla zadania omijania przeszkody
% wyjscie:
% J -- jakobian (2*n)
% wejscie:
% q -- wspolrzedne zlaczowe (n*1)
% l -- dlugosci czlonow (n*1)

    if (length(q) ~= length(l))
        disp("obstacleJacobian: q i l maja rozna dlugosc");
        return
    end

    n = length(q);
    JO = zeros(2, n);
    % punkt omijajacy przeszkody jest na lokciu robota (na koncu drugiego
    % czlonu)
    for j = 1:2
        for i = j:n
            JO(1, j) = JO(1, j) - l(i) * sin( sum( q(1:i) ) );
            JO(2, j) = JO(2, j) + l(i) * cos( sum( q(1:i) ) );
        end
    end

end