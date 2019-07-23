% bar_main.m

clearvars; close all; clc;

% Build a thr struct to use in place of the class for now
% For the class I'll make two constructors: one build up from links and one
% build down from tether. todo(rodney)
thr.numlinks = 10;
thr.length = 12; % m
thr.meanDiameter = 0.1; % m
thr.mass = 3; % kg
thr.density = 100;
env.gravity = 9.8;
env.fluid.density = 1000;
for i=1:1:thr.numlinks
    thr.link(i).length = thr.length/thr.numlinks; % m
    thr.link(i).diameter = thr.meanDiameter; 
    thr.link(i).mass = thr.mass/thr.numlinks;   % kg
    thr.link(i).reldens = thr.density/env.fluid.density;
    thr.link(i).weight = pi/4*thr.link(i).diameter^2*thr.link(i).length*env.gravity*(thr.link(i).reldens-1);
    temp = thr.link(i).mass*(thr.link(i).length^2 + 3/4*thr.link(i).diameter^2)/12;
    thr.link(i).inertia = [1/8*thr.link(i).mass*thr.link(i).diameter^2 0 0; 0 temp 0; 0 0 temp];
    clear temp
    thr.link(i).x = 0.1;
    thr.link(i).c = thr.link(i).length - thr.link(i).x;
    thr.link(i).moment = [0;0;0];
    % Initial states
    qj = [1;4;3;5];
    qj = qj/norm(qj);
    x0(3*i-2,1) = 0;
    x0(3*i-1,1) = 0;
    x0(3*i,1) = 0;
    x0(3*thr.numlinks+4*i-3,1) = qj(1);
    x0(3*thr.numlinks+4*i-2,1) = qj(2);
    x0(3*thr.numlinks+4*i-1,1) = qj(3);
    x0(3*thr.numlinks+4*i,1) = qj(4);
end
thr.endForce = [0;0;0];
thr.baseMoment = [0;0;0];

t = 0;
xdot = bar_state(t,x0,thr);
simtime = 0:0.001:30;
opts = odeset('RelTol',1e-5,'Stats','on','OutputFcn',@odeplot);
[time,y] = ode45(@(t,x) bar_state(t,x,thr),simtime,x0,opts);