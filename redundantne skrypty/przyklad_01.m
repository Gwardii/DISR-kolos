% DiSR 
% Przyklad 1: pseudoodwrotnosc Moore'a-Penrose'a
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

% pseudoodwrotnosci jakobianu
Jp1 = pinv(J);
Jp2 = J'*inv(J*J'); % - dzia³a kiedy jakobian ma pe³ny rz¹d, brak osobliwoœci
[U,S,V] = svd(J); %U - psi, S - sigma, V - ni  
Jp3 = V * pinv(S) * U';