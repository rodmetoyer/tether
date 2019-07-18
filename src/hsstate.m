function xdot = hsstate(t,x,thr,env)
% hsstate
% computes the state derivative vector for an n-link tether using the
% Hembree-Slegers recursive bar model.

% constants (todo replace with tether)
numLinks = 5;
tetherMass = 5; % kg
tetherLegth = 100; % m
m = tetherMass/numLinks; % Params should be row arrays. For prototype everything is homogeneous so one param will do.
linkLgth = tetherLegth/numLinks; % m
xm = linkLgth/2; % Assuming cm is at center of rod
Iyy = m*linkLgth^2/12;
Izz = Iyy;
I = [Iyy 0; 0 Izz];
Stild = [0 -xm;xm 0];
Shat = [0 0 -xm; 0 xm 0];

%% Start at the terminal link
% Normalize Euler Parameters and build B_C_O matrics
% qmag = sqrt(x(10)^2 + x(11)^2 + x(12)^2 + x(13)^2);
% x(10) = x(10)/qmag; x(11) = x(11)/qmag; x(12) = x(12)/qmag; x(13) = x(13)/qmag; 