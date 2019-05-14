% tetherSimDriver.m
% This is the driver for the tether simulations.
% You have the choice of running the simulations from this script by
% setting 'runFromScript' to true and varying default parameter values
% manually. You can also just run the script and use the popup windows to
% run the simulation, or you can choose input files.

% This code was written by Rodney Metoyer and Chris Yoder

% Setup workspace
clearvars; close all; clc;
addpath('src');

% Setup default simulation parameters.
% We'll also use these for running directly from the script
runFromScript = false; % Make true to skip GUI choices - todo make funcitonal
simP.totaltime = 5;
simP.timestep = 0.001;
tetherP.length = 10.0;
tetherP.mass = 0.1;
tetherP.radius = 1;
tetherP.springk = 150;
tetherP.dampFac = 1.5;
tetherP.relativeDensity = 2.0;
tetherP.numNodes = 20;

% Choice of one or more input files or manual simulation set-up
opts.Interpreter = 'tex'; opts.Default = 'Input File(s)'; opts.Resize='on';
answer = questdlg('\fontsize{12}\color{black}Manual simulation setup or choose input file(s)?','Simulation Setup','Manual','Input File(s)',opts);
if strcmp(answer,'Manual')
    % Use inputdlg to get all the simulation inputs
    answer = inputdlg({'\fontsize{10}Enter simulation duration', '\fontsize{10}Enter simulation time step'},...
        'Simulation Parameters',[1 50;1 50],{num2str(simP.totaltime),num2str(simP.timestep)},opts);
    simP.totaltime = str2double(answer(1));
    simP.timestep = str2double(answer(2));
    answer = inputdlg({'\fontsize{10}Enter tether length', '\fontsize{10}Enter tether mass',...
        '\fontsize{10}Enter tether radius','\fontsize{10}Enter tether spring constant','\fontsize{10}Enter tether damping factor',...
        '\fontsize{10}Enter tether relative density','\fontsize{10}Enter number of nodes'},'Tether Parameters',[1 50;1 50;1 50;1 50;1 50;1 50;1 50],...
        {num2str(tetherP.length),num2str(tetherP.mass),num2str(tetherP.radius),...
        num2str(tetherP.springk),num2str(tetherP.dampFac),num2str(tetherP.relativeDensity),num2str(tetherP.numNodes)},opts);
    tetherP.length = str2double(answer(1));
    tetherP.mass = str2double(answer(2));
    tetherP.radius = str2double(answer(3));
    tetherP.springk = str2double(answer(4));
    tetherP.dampFac = str2double(answer(5));
    tetherP.relativeDensity = str2double(answer(6));
    tetherP.numNodes = str2double(answer(7));
    % Setup simulation
    
    % Run simulation
    
    % Process Results (dump data to file)
    
elseif strcmp(answer,'Input File(s)')
    % Get the simulation inputs directly from the files
    % Putting this in because we need to support something other than .mat
    % files if we want to write the code in another language. But for now
    % it's just easiest to load the mat files directly.
    opts.Default = '.txt';
    answer = questdlg('\fontsize{12}\color{black}What tpye of input file do you want to use?','Input file type','.txt','.mat',opts);
    extension = '.txt';
    if strcmp(answer,'.mat')
        uiwait(msgbox('Psych! Only .txt currently supported. That"s what you"re going to use.','Just kidding message.','modal'));
        % extension = '.mat';
    elseif strcmp(answer,'.txt')
        extension = '.txt';
    end
    pth = [pwd '\input'];
    inputfiles = getFilesList(pth,extension);
    [indx,tf] = listdlg('ListString',inputfiles,'PromptString','Select data files to process');
    % todo(rodney) if ~tf then (user clicked cancel) run the entire batch?
    inputfiles = inputfiles(indx);
    % Run the simulation for every input file chosen
    for i=1:1:numel(inputfiles)
        % Load the inputs
        fid = fopen([pth '\' char(inputfiles(i))]);
        while true
            tline = fgetl(fid);            
            if isnumeric(tline)
                break;
            end
            eval(tline);
        end
        % Setup the simulation
        
        % Run the simulation
        
        % Process the results
        % Let's dump the data to a data folder in a subfolder the same name
        % as the input file.
    end
end

% Postprocess (as many times as chosen by user - i.e. goto data folder adn
% allow user to select which data to post)
    % Load data from file 
    % Make plots and movies

% Setup simulation
simTimes = 0:simP.timestep:simP.totaltime;
numSteps = numel(simTimes);
makemovie = true;
moviefile = 'junk.avi';
tiptrails = true;
frmrt = 40; % Framerate right here

% In the future you will be able to build a tether from links and nodes.
% For now the links and nodes get built for you. Instantiate a kvlink
% lj = kvlink(1000,25,2,0.1,0.025);

% Make a tether with some kv links
thr = tether(tetherP.length,tetherP.mass,tetherP.radius,tetherP.springk,tetherP.dampFac,tetherP.relativeDensity,tetherP.numNodes);

% Alter node position prior to state vector initialization
theta1 = -90;
theta2 = 0;
positions = NaN(3,tetherP.numNodes); speeds = NaN(3,tetherP.numNodes);
for i=1:1:tetherP.numNodes
    positions(:,i) = [tetherP.length/(tetherP.numNodes-1)*(i-1)*cosd(theta1)*cosd(theta2);tetherP.length/(tetherP.numNodes-1)*(i-1)*cosd(theta1)*sind(theta2);tetherP.length/(tetherP.numNodes-1)*(i-1)*sind(theta1)];
    speeds(:,i) = [0;0;0];
end        
thr.setNodeStates(positions,speeds);
% Adjust individual nodes (ex. last node) - todo add dlg
thr.nodes(tetherP.numNodes).setNodeMass(0.2);

% Initial states
% In the current implementation the state vector and the tether node states
% are coupled only through the parameters (node states are not ODE states);
for i=1:1:tetherP.numNodes
    x0(3*i-2) = thr.nodes(i).x;
    x0(3*i-1) = thr.nodes(i).y;
    x0(3*i) = thr.nodes(i).z;
    x0(3*tetherP.numNodes+3*i-2) = 0;
    x0(3*tetherP.numNodes+3*i-1) = 0;
    x0(3*tetherP.numNodes+3*i) = 0;
end
disp('Starting simulation...');
tic
opts = odeset('RelTol',1e-5,'AbsTol',1e-7,'Stats','on');%,'OutputFcn',@odeplot);
[time,svec] = ode45(@(t, x)kvstate(t, x, thr),simTimes,x0,opts);


toc
for i=1:1:tetherP.numNodes
    x(:,i) = svec(:,3*i-2);
    y(:,i) = svec(:,3*i-1);
    z(:,i) = svec(:,3*i);
    u(:,i) = svec(:,3*tetherP.numNodes+3*i-2);
    v(:,i) = svec(:,3*tetherP.numNodes+3*i-1);
    w(:,i) = svec(:,3*tetherP.numNodes+3*i);
end

%% Movie
if makemovie
hfig = figure('Color','w','Position',[100 100 1000 700]);
cmap = jet(tetherP.numNodes);
numtrail = 50;
trmap = gray(numtrail);
for i=1:1:frmrt*simP.totaltime+1
    n=(i-1)*(numSteps-1)/(frmrt*simP.totaltime)+1;
    plot3(0,0,0,'*k'); hold on;
    %plot3(x(n,:),y(n,:),z(n,:),'o','Color',cmap(j,:),'MarkerFaceColor',cmap(j,:));
    for j=2:1:tetherP.numNodes
        plot3([x(n,j-1) x(n,j)],[y(n,j-1) y(n,j)],[z(n,j-1) z(n,j)],'-o','Color',cmap(j,:),'LineWidth',2.0,'MarkerSize',3,'MarkerFaceColor',cmap(j,:));
    end
    a = 1.1; b = 1;
    plot3([x(n,j) x(n,j)-a*sin(b*time(n))],[y(n,j) y(n,j)-2*a*sin(2*b*time(n))],[z(n,j) z(n,j)+0.0],'b-');
    if tiptrails
    clear nprev
        for j=1:1:min(i,numtrail)
            nprev = (i-j)*(numSteps-1)/(frmrt*simP.totaltime)+1;
            plot3(x(nprev,tetherP.numNodes),y(nprev,tetherP.numNodes),z(nprev,tetherP.numNodes),'.','Color',trmap(j,:));
        end
    end
    
    grid on
    axis equal; axis([-1.1*tetherP.length,1.1*tetherP.length,-1.1*tetherP.length,1.1*tetherP.length,-1.1*tetherP.length,1.1*tetherP.length]);
    xlabel('x(m)');ylabel('y(m)');zlabel('z(m)');
    title(['Time: ' num2str(time(n),'%4.2f')]);
    hold off
    M(i) = getframe(hfig);
end
moviefile = ['images\' moviefile];
writerObj = VideoWriter(moviefile);
writerObj.FrameRate = frmrt; writerObj.Quality = 100; % optional
open(writerObj); writeVideo(writerObj,M); close(writerObj);
end