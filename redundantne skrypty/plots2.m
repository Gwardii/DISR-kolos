function plots2()
% DiSR
% Funkcja do rysowania wykresow, wariant 2

load('wyniki.mat', 'Q', 'dQdt', 'd2Qdt2', 'X', 'V', 'ErrNorm', 'T', 'H');

tiledlayout(2,3)
nexttile
plot(T, Q/pi*180); legend("1", "2", "3", "4"); xlabel("t, s"); ylabel("q, deg"); grid on;
nexttile
plot(T, dQdt/pi*180); legend("1", "2", "3", "4"); xlabel("t, s"); ylabel("dq/dt, deg/s"); grid on;
nexttile
plot(T, d2Qdt2/pi*180); legend("1", "2", "3", "4"); xlabel("t, s"); ylabel("d2q/dt2, deg/s^2"); grid on;
nexttile
plot(T, X); legend("x", "y"); xlabel("t, s"); ylabel("r, m"); grid on;
nexttile
plot(T, H); xlabel("t, s"); ylabel("H(q)"); grid on;
nexttile
plot(T, ErrNorm);  xlabel("t, s"); ylabel("||e||, m"); grid on;

end