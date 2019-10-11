% pnodetest.m

clearvars; close all; clc;
addpath('..\src');
% I add to the path because of the way I stucture my files. You don't need
% to do this if pnode is in the same folder as your other code.

% When you instantiate the pnode you can either pass the spring constant,
% the damping constant, and the path shape or you can instantiate with no
% arguments and get the defaults.
pn1 = pnode(1000,250,'eight');
pn2 = pnode;
pn3 = pnode(1000,250,'ellipse');

% When you want the position for the given path shape call getPosition with
% the time and optional shape parameters.
duration = 2;
timestep = 0.02;
time = 0:timestep:duration;
% I'll make a video to show what it is doing
hfig = figure('Color','w');
for i=1:1:length(time)
    % getPosition returns a 2D vector of positions in the plane
    width = 2; height = 1; speed = pi;
    posvec2D = pn1.getPosition(time(i),width,height,speed);
    % Make the second one move faster
    speed = 1.5*speed;
    pvec2 = pn2.getPosition(time(i),width,height,speed);
    % mess with the shape for the ellipse one
    width = 2; height = 3; speed = 1.0*pi;
    pvec3 = pn3.getPosition(time(i),width,height,speed);
    plot(posvec2D(1),posvec2D(2),'.r',pvec2(1),pvec2(2),'.b',pvec3(1),pvec3(2),'.k','MarkerSize',10.0);
    axis equal
    axis([-(width+0.1*width) width+0.1*width -(height+0.1*height) height+0.1*height]);
    title(['Time (s) = ' num2str(time(i),'%4.2f');]);
    hold on % uncomment if you want the points to persist
    M(i) = getframe(hfig);
end
% For now compute the force in the state function by:
% posVec = [pnode.getPosition constantZ] if you are in the x-y plane
% diffVec = posVec - endNodeVec;
% glueForceSpring = diffVec*pnode.springk;
% You probably don't need the damping, but I'll double check when I get the
% chance. 