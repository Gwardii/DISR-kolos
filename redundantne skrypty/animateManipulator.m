function animateManipulator()
% DiSR
% Funkcja do animowania ruchu manipulatora

% wymiary manipulatora
l = [0.25 0.25 0.25 0.25];

% przebiegi wspolrzednych zlaczowych w koljenych krokach
load('wyniki.mat', 'Q');
[N, n] = size(Q);
if (n ~= length(l))
	disp("animateManipulator: q i l maja rozna dlugosc");
	return
end

% ile klatek narysowac
Nframes = 200;
if Nframes > N
    Nframes = N;
end
deltaFrames = floor(N/Nframes);

% wymiary okienka do rysowania
xMin = -0.5;
xMax = 1;
yMin = -0.5;
yMax = 1;

% tablica na polozenia efektora
EE = [];

for i = 1:deltaFrames:N
	X = forwardKinematicsAnimate(Q(i,:)', l);
    if i == 1
        EE = X(:,n+1);
    else
        EE = [EE X(:,n+1)];
    end
	h = plot(EE(1,:), EE(2,:), X(1,:), X(2,:));
	h(1).LineWidth = 2;
    h(1).Color = 'red';
	h(2).LineWidth = 4;
    h(2).Color = 'blue';
	grid on;
	axis([xMin xMax yMin yMax]);
    xlabel("x, m");
    ylabel("y, m");
	pause(0.1);
	drawnow
% 	
% 	  % Capture the plot as an image 
%       frame = getframe(1); 
%       im{i} = frame2im(frame); 
%       [imind,cm] = rgb2ind(im{i},256); 
% 
% %       Write to the GIF File 
%       if i == 1 
%           imwrite(imind,cm,'planar.gif','gif', 'Loopcount',inf); 
%       else 
%           imwrite(imind,cm,'planar.gif','gif','WriteMode','append'); 
%       end 
% 	
end

end

function X = forwardKinematicsAnimate(q, l)
% wejscie:
% q -- wektor wspolrzednych zlaczowych (rad)
% l -- dlugosci czlonow manipulatora (m)
% wyjscie:
% X -- wspolrzedne poczatkow czlonow (m)
% X = [poczatek_czlonu_1 ... poczatek_czlonu_n polozenie_efektora]

n = length(q);
x = zeros(2, 1);
X = zeros(2, n + 1);
for j = 1:n
   x(1) = x(1) + l(j) * cos( sum( q(1:j) ) );
   x(2) = x(2) + l(j) * sin( sum( q(1:j) ) );
   X(:, j + 1) = x;
end

end