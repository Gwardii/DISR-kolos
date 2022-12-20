function plots3()
% DiSR
% Funkcja do rysowania wykresow, wariant 3

load('wyniki.mat', 'Q', 'dQdt', 'd2Qdt2', 'X', 'V', 'ErrNorm', 'T', 'D');

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
plot(T, D); xlabel("t, s"); ylabel("distance to obstacle, m"); grid on;
nexttile
plot(T, ErrNorm);  xlabel("t, s"); ylabel("||e||, m"); grid on;

end