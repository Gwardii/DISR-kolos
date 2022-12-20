% DiSR 
% Przyklad 2: pseudoodwrotnosc Moore'a-Penrose'a
clear; clc;
% wymiary manipulatora
l = [0.25 0.25 0.25 0.25];

% konfiguracja zlaczowa
q = [45; -90; 90; -90] / 180 * pi;

% polozenie efektora odpowiadajace q
X = forwardKinematics(q, l);

% jakobian odpowiadajacy q
J = jacobian2DOF(q, l);

% wymiary jakobianu
[m, n] = size(J);

% rzad jakobianu
r = rank(J);

% pseudoodwrotnosci jakobianu (Moore'a - Penrose'a)
Jp1 = pinv(J);
Jp2 = J'*inv(J*J'); % - dzia³a kiedy jakobian ma pe³ny rz¹d, brak osobliwoœci
[U,S,V] = svd(J); %U - psi, S - sigma, V - ni  
Jp3 = V * pinv(S) * U';

% predkosc efektora
v = [1; 1]; % m/s

% predkosci zlaczowe
dqdt1 = Jp1 * v; % pinv
dqdt2 = Jp2 * v; % wzor (9)
dqdt3 = Jp3 * v; % SVD
dqdt4 = J \ v; % \

% zadanie proste kinematyki o predkosciach
v1 = J * dqdt1;
v2 = J * dqdt2;
v3 = J * dqdt3;
v4 = J * dqdt4;