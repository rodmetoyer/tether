function xdot = kvstate(t,x,thr,fterm)
% fterm is the terminal load in the inertial frame
% Plan is to pass fterm into the tether module from the vehicle module
% For developing lets make fterm some funciton of time
% todo(rodney) replace fterm with an environment object. That class will
% hold fluid info from which a force may be calculated as well as gravity
% info.
%
% INPUTS:
%   t       [s] current sim time
%   x       state vector
%   thr     tether object
%   fterm   terminal load
%
% OUTPUTS:
%   xdot    state derivative vector

% x0 = [x1, y1, z1, x2, ... zn, u1, v1, w1, u2, ..., wn]


if nargin < 4                                   % if no fterm specified, add one
    %fterm = [0;0;0];
    a = 0.1; b = 1;                             % coeffs to make figure 8
    fterm = [-a*sin(b*t);-2*a*sin(2*b*t);0.0];  % fterm is the terminal load in the tether frame
end

numnodes = numel(thr.nodes);    % number of nodes
xdot = NaN(6*numnodes,1);       % preallocate state derivative vector
gravity = 9.81;                 % todo make this come from the enviroment object.

% Update first 3*numnodes elements in xdot
xdot(1:3*numnodes) = x(3*numnodes+1:end);   % set u, v, w to xd, yd, zd

% First node
% The first node is geometrically constrained
r = [x(4);x(5);x(6)]-[x(1);x(2);x(3)];      % calculate vector r_2/1_O
lr = norm(r);                               % get magnitude of vector r_2/1_O
unitr = r/lr;                               % calculate unit vector of r_2/1_O
stretch = lr-thr.links(1).baselength;       % calculate stretch = r_2/1_O - L0
flink = [0;0;0];                            % initialize force on link
v = [x(3*numnodes+4);x(3*numnodes+5);x(3*numnodes+6)] - [x(3*numnodes+1);x(3*numnodes+2);x(3*numnodes+3)];
stretchd = (r(1)*v(1)+r(2)*v(2)+r(3)*v(3))/lr;  % correct equation
if stretch > 0
    flink = (stretch*thr.links(1).stiffness + stretchd*thr.links(1).damping)*unitr;
end

% Add buoyancy
% ftot = flink + [0;0;thr.nodes(1).buoyancy];

% Add drag - none needed for the current implementation, but in general the
% first node may not be constrained. Need to fgure out if I am going to
% make that a law or not once I get the implementation ironed out.

% Update acceleration (remember gravity) - first node constrained
xdot(3*numnodes+1) = 0; % ftot(1)/thr.nodes(1).mass;
xdot(3*numnodes+2) = 0; % ftot(2)/thr.nodes(1).mass;
xdot(3*numnodes+3) = 0; % ftot(3)/thr.nodes(1).mass-gravity;

% Internal nodes
fprevlink = flink; % force on the previous node
for i=2:1:numnodes-1
    r = [x(3*i+1);x(3*i+2);x(3*i+3)]-[x(3*i-2);x(3*i-1);x(3*i)];    % calculate r_n+1/n_O vector
    lr = norm(r);                                                   % calculate norm r_n+1/n_O vector
    unitr = r/lr;
    stretch = lr-thr.links(i).baselength;
    flink = [0;0;0];
    v = [x(3*numnodes+3*i+1);x(3*numnodes+3*i+2);x(3*numnodes+3*i+3)] - [x(3*numnodes+3*i-2);x(3*numnodes+3*i-1);x(3*numnodes+3*i)];
    stretchd = (r(1)*v(1)+r(2)*v(2)+r(3)*v(3))/lr;
    if stretch > 0
        flink = (stretch*thr.links(i).stiffness + stretchd*thr.links(i).damping)*unitr;
    end
    
    % Add buoyancy and previous link
    ftot = flink + [0;0;thr.nodes(i).buoyancy] - fprevlink;
    
    % Add drag
    
    % Update acceleration (remember gravity) - first node constrained
    xdot(3*numnodes+3*i-2) = ftot(1)/thr.nodes(i).mass;
    xdot(3*numnodes+3*i-1) = ftot(2)/thr.nodes(i).mass;
    xdot(3*numnodes+3*i) = ftot(3)/thr.nodes(i).mass-gravity;
    fprevlink = flink; % update force on the node from the previous line
end

% Add buoyancy and previous link and fterm
ftot = [0;0;thr.nodes(numnodes).buoyancy] - fprevlink + fterm;

% Add drag


% Update acceleration (remember gravity) - first node constrained
xdot(3*numnodes+3*numnodes-2) = ftot(1)/thr.nodes(numnodes).mass;
xdot(3*numnodes+3*numnodes-1) = ftot(2)/thr.nodes(numnodes).mass;
xdot(3*numnodes+3*numnodes) = ftot(3)/thr.nodes(numnodes).mass-gravity;

end % end function kvstate