% bar_main.m

clearvars; close all; clc;

% sim setup
sim.Duration = 60;
sim.TimeStep = 0.01;

% Results visualization setup
makemovie = true; moviefile = 'FiveLinks_noDrag.avi'; frmrt = 10;

% Build a thr struct to use in place of the class for now
% For the class I'll make two constructors: one build up from links and one
% build down from tether. todo(rodney)
thr.numlinks = 5;
thr.length = 12; % m
thr.meanDiameter = 0.1; % m
thr.mass = 3; % kg
thr.density = 900;
env.gravity = 9.8;
env.fluid.density = 1000;
for i=1:1:thr.numlinks
    thr.link(i).length = thr.length/thr.numlinks; % m
    thr.link(i).diameter = thr.meanDiameter; 
    thr.link(i).mass = thr.mass/thr.numlinks;   % kg
    thr.link(i).reldens = thr.density/env.fluid.density;
    thr.link(i).weight = -pi/4*thr.link(i).diameter^2*thr.link(i).length*env.gravity*(thr.link(i).reldens-1); % if more dense weight is down
    temp = thr.link(i).mass*(thr.link(i).length^2 + 3/4*thr.link(i).diameter^2)/12;
    thr.link(i).inertia = [1/8*thr.link(i).mass*thr.link(i).diameter^2 0 0; 0 temp 0; 0 0 temp];
    clear temp
    thr.link(i).x = 0.1;
    thr.link(i).c = thr.link(i).length - thr.link(i).x;
    thr.link(i).moment = [0;0;0];
    % Initial states
    qj = [1;0;0;0];
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
simtime = 0:sim.TimeStep:sim.Duration; 
opts = odeset('RelTol',1e-5,'Stats','on','OutputFcn',@odeplot);
[time,y] = ode45(@(t,x) bar_state(t,x,thr),simtime,x0,opts);

%% Post process
% set-up
N = thr.numlinks;
numSteps = length(y);
for j=1:1:numSteps
    for i = 1:1:N
        qj = [y(j,3*N+4*i-3);y(j,3*N+4*i-2);y(j,3*N+4*i-1);y(j,3*N+4*i)];
        j_C_O = [qj(1)^2+qj(2)^2-qj(3)^2-qj(4)^2, 2*(qj(2)*qj(3) + qj(1)*qj(4)),   2*(qj(2)*qj(4) - qj(1)*qj(3));...
                 2*(qj(2)*qj(3) - qj(1)*qj(4)),   qj(1)^2-qj(2)^2+qj(3)^2-qj(4)^2, 2*(qj(3)*qj(4) + qj(1)*qj(2));...
                 2*(qj(2)*qj(4) + qj(1)*qj(3)),   2*(qj(3)*qj(4) - qj(1)*qj(2)),   qj(1)^2-qj(2)^2-qj(3)^2+qj(4)^2];
        O_C_j = transpose(j_C_O);    
        if i==1
            r_co_O(3*i-2:3*i,j) = O_C_j*[thr.link(i).length;0;0];            
        else
            r_co_O(3*i-2:3*i,j) = O_C_j*[thr.link(i).length;0;0] + r_co_O(3*(i-1)-2:3*(i-1),j);
        end
        link(i).position(:,j) = r_co_O(3*i-2:3*i,j);
        linkx(i,j) = r_co_O(3*i-2,j);
        linky(i,j) = r_co_O(3*i-1,j);
        linkz(i,j) = r_co_O(3*i,j);
    end % end post set-up loop
end % end time loop
% Add the ground node for plotting
linkx = [zeros(1,length(linkx));linkx];
linky = [zeros(1,length(linky));linky];
linkz = [zeros(1,length(linkz));linkz];

cmap = jet(N);
% Look at the evolution of the states
for i=1:1:N
    figure('Position',[100 100 800 600]);
    plot(time,y(:,3*i-2),'Marker','+','Color','r');%cmap(i,:));
    hold on
    plot(time,y(:,3*i-1),'Marker','o','Color','b');%cmap(i,:));
    plot(time,y(:,3*i),'Marker','*','Color','g');%cmap(i,:));
    hold off
    title(['Link ' num2str(i)]);
    xlabel('time (s)'); ylabel('angular rate (rad/s)');
end

for i=1:1:N
    figure('Position',[600 400 800 600]);
    plot(time,y(:,3*N+4*i-3),'Marker','+','Color','r');%cmap(i,:));
    hold on
    plot(time,y(:,3*N+4*i-2),'Marker','o','Color','b');%cmap(i,:));
    plot(time,y(:,3*N+4*i-1),'Marker','*','Color','g');%cmap(i,:));
    plot(time,y(:,3*N+4*i),'Marker','^','Color','k');%cmap(i,:));
    hold off
    title(['Link ' num2str(i)]);
    xlabel('time (s)'); ylabel('Euler Parameter');
end

cmap = jet(N+1);
% Start position
figure
scatter3(linkx(:,1),linky(:,1),linkz(:,1),20,cmap,'filled');

% mid position
figure
timeloc = round(0.5*length(simtime));
scatter3(linkx(:,timeloc),linky(:,timeloc),linkz(:,timeloc),20,cmap,'filled');

% End position
figure
scatter3(linkx(:,end),linky(:,end),linkz(:,end),20,cmap,'filled');

% Movie
if makemovie
hfig = figure('Color','w','Position',[100 100 1000 700]);
% numtrail = 50;
% trmap = gray(numtrail);
for i=1:1:frmrt*sim.Duration+1
    n=(i-1)*(numSteps-1)/(frmrt*sim.Duration)+1;
    for j=1:1:N
        plot3([linkx(j,n) linkx(j+1,n)],[linky(j,n) linky(j+1,n)],[linkz(j,n) linkz(j+1,n)],'color',cmap(j,:));
        hold on;
    end    
    grid on
    axis equal; axis([-1.1*thr.length,1.1*thr.length,-1.1*thr.length,1.1*thr.length,-1.1*thr.length,1.1*thr.length]);
    xlabel('x(m)');ylabel('y(m)');zlabel('z(m)');
    title(['Time: ' num2str(time(n),'%4.2f')]);
    hold off
    M(i) = getframe(hfig);
end
% todo do we want to add a dlg to choose a folder to save to?
if ~exist([pwd '\products'],'dir')
    mkdir('products');
end
if ~exist([pwd '\products\videos'],'dir')
    mkdir('products\videos');
end
moviefile = ['products\videos\' moviefile];
writerObj = VideoWriter(moviefile);
writerObj.FrameRate = frmrt; writerObj.Quality = 100; % optional
open(writerObj); writeVideo(writerObj,M); close(writerObj);
end
