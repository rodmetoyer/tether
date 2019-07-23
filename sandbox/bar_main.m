% bar_main.m

clearvars; close all; clc;

% Build a thr struct to use in place of the class for now
thr.numlinks = 10;
for i=1:1:thr.numlinks
    thr.link(i).length = 0.2; % m
    thr.link(i).mass = 0.3;   % kg
    temp = thr.link(i).mass*thr.link(i).length*thr.link(i).length/12;
    thr.link(i).inertia = [0 0 0; 0 temp 0; 0 0 temp];
    clear temp
    thr.link(i).x = 0.1;
    thr.link(i).c = thr.link(i).length - thr.link(i).x;
    thr.link(i).moment = [0;0;0];
    % Initial states
    qj = [1;4;3;5];
    qj = qj/norm(qj);
    x0(7*i-6,1) = 0;
    x0(7*i-5,1) = 0;
    x0(7*i-4,1) = 0;
    x0(7*i-3,1) = qj(1);
    x0(7*i-2,1) = qj(2);
    x0(7*i-1,1) = qj(3);
    x0(7*i,1) = qj(4);
end
thr.endForce = [0;0;0];
thr.baseMoment = [0;0;0];

t = 0;
xdot = bar_state(t,x0,thr);