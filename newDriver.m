% newDriver.m
% I'm refactoring. Don't use this.

% Setup workspace
clearvars; close all; clc;
addpath('src');

% I want to be able to run simulations by either passing an input file or
% manually setting up the simulation. The idea is that we can do something
% like this:
inputfiles = ["sim1.txt"];
pth = [pwd '\input'];
for i=1:1:numel(inputfiles)
    % Make a simulation object with default values. You can also initialize
    % with name-value pairs.
    sim = simulation;
    % Use the parseInputFile method to parse the input file and set-up the
    % simulation.
    sim.parseInputFile([pth '\' char(inputfiles(i))]);
    
    % At this point all the objects should be instantiated, so its just:
    [time,states] = sim.run([]);
    [x,y,z,u,v,w] = breakoutStates(states);
    % Send to file
    
end

frame = 6501;
[~,n] = size(states); nn = n/6;
cmap = jet(nn);
figure
plot3(0,0,0,'*k'); hold on;
axis equal
for j=2:1:nn
    plot3([x(frame,j-1) x(frame,j)],[y(frame,j-1) y(frame,j)],[z(frame,j-1) z(frame,j)],...
        '-o','Color',cmap(j,:),'LineWidth',2.0,'MarkerSize',3,'MarkerFaceColor',cmap(j,:));
end
hold off;

function [x,y,z,u,v,w] = breakoutStates(states)
    [~,n] = size(states);
    nn = n/6;
    for i=1:1:nn
        x(:,i) = states(:,6*i-5);
        y(:,i) = states(:,6*i-4);
        z(:,i) = states(:,6*i-3);
        u(:,i) = states(:,6*i-2);
        v(:,i) = states(:,6*i-1);
        w(:,i) = states(:,6*i);
    end
end