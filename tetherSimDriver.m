% test.m
% test the tether code base as I develop

% Setup workspace
clearvars; close all; clc;
addpath('src');

% Setup simulation
sim.totaltime = 10;
sim.timestep = 0.001;
simTimes = 0:sim.timestep:sim.totaltime;
numSteps = numel(simTimes);
makemovie = true;
moviefile = 'tonsofnodes.avi';
frmrt = 40;

% Instantiate a kvlink
% lj = kvlink(1000,25,2,0.1,0.025);

% Make a tether with some kv links
length = 10.0;
mass = 0.1;
radius = 1;
springk = 150;
dampFac = 1;
relativeDensity = 0.7;
numNodes = 50;
thr = tether(length,mass,radius,springk,dampFac,relativeDensity,numNodes);

% Alter node position prior to state vector initialization
theta1 = -20;
theta2 = 135;
positions = NaN(3,numNodes); speeds = NaN(3,numNodes);
for i=1:1:numNodes
    positions(:,i) = [length/(numNodes-1)*(i-1)*cosd(theta1)*cosd(theta2);length/(numNodes-1)*(i-1)*cosd(theta1)*sind(theta2);length/(numNodes-1)*(i-1)*sind(theta1)];
    speeds(:,i) = [0;0;0];
end        
thr.setNodeStates(positions,speeds);

% Initial states
% In the current implementation the state vector and the tether node states
% are coupled only through the parameters (node states are not ODE states);
for i=1:1:numNodes
    x0(3*i-2) = thr.nodes(i).x;
    x0(3*i-1) = thr.nodes(i).y;
    x0(3*i) = thr.nodes(i).z;
    x0(3*numNodes+3*i-2) = 0;
    x0(3*numNodes+3*i-1) = 0;
    x0(3*numNodes+3*i) = 0;
end
disp('Starting simulation...');
tic
opts = odeset('RelTol',1e-5,'AbsTol',1e-7,'Stats','on');%,'OutputFcn',@odeplot);
[time,svec] = ode45(@(t, x)kvstate(t, x, thr),simTimes,x0,opts);
toc
for i=1:1:numNodes
    x(:,i) = svec(:,3*i-2);
    y(:,i) = svec(:,3*i-1);
    z(:,i) = svec(:,3*i);
    u(:,i) = svec(:,3*numNodes+3*i-2);
    v(:,i) = svec(:,3*numNodes+3*i-1);
    w(:,i) = svec(:,3*numNodes+3*i);
end

%% Movie
if makemovie
hfig = figure('Color','w','Position',[100 100 1000 700]);
cmap = jet(numNodes);
for i=1:1:frmrt*sim.totaltime+1
    n=(i-1)*(numSteps-1)/(frmrt*sim.totaltime)+1;
    plot3(0,0,0,'*k'); hold on;
    %plot3(x(n,:),y(n,:),z(n,:),'o','Color',cmap(j,:),'MarkerFaceColor',cmap(j,:));
    for j=2:1:numNodes
        plot3([x(n,j-1) x(n,j)],[y(n,j-1) y(n,j)],[z(n,j-1) z(n,j)],'-o','Color',cmap(j,:),'LineWidth',2.0,'MarkerSize',3,'MarkerFaceColor',cmap(j,:));
    end
    axis equal; axis([-1.1*length,1.1*length,-1.1*length,1.1*length,-1.1*length,1.1*length]);
    xlabel('x(m)');ylabel('y(m)');zlabel('z(m)');
    title(['Time: ' num2str(time(n),'%4.2f')]);
    hold off
    M(i) = getframe;
end

writerObj = VideoWriter(moviefile);
writerObj.FrameRate = frmrt; writerObj.Quality = 100; % optional
open(writerObj); writeVideo(writerObj,M); close(writerObj);
end