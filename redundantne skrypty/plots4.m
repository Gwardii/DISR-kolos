function plots4()
% DiSR
% Funkcja do rysowania wykresow, wariant 4

load('wyniki.mat', 'Q', 'dQdt', 'd2Qdt2', 'X', 'V', 'ErrNorm', 'T', 'S', 'S_mod');

tiledlayout(2,3)
nexttile
plot(T, Q/pi*180); legend("1", "2", "3", "4"); xlabel("t, s"); ylabel("q, deg"); grid on;
nexttile
plot(T, dQdt/pi*180); legend("1", "2", "3", "4"); xlabel("t, s"); ylabel("dq/dt, deg/s"); grid on;
nexttile
plot(T, d2Qdt2/pi*180); legend("1", "2", "3", "4"); xlabel("t, s"); ylabel("d2q/dt2, deg/s^2"); grid on;
nexttile
plot(T, S); legend("\sigma_1", "\sigma_2"); xlabel("t, s"); ylabel("\sigma"); grid on;
nexttile
plot(T, S_mod); legend("\sigma_1", "\sigma_2"); xlabel("t, s"); ylabel("\sigma_{mod}"); grid on;
nexttile
plot(T, ErrNorm);  xlabel("t, s"); ylabel("||e||, m"); grid on;

end