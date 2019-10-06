classdef simulation < handle
    % simulation class manages the simulation
    
    properties
        duration % duration of the simulation
        timestep % the time step for the ODE solver
        solver   % the type of ODE solver        
    end
    
    % These should be tethersim properties. I'll make it later.
    properties
        tether       % one (or more?) tether objects
        environment  % the environment that the tether is in
    end    
    
    methods
        % Constructor
        function hobj = simulation(varargin)
            % Creates a simulation object to control the simulation
            % INPUTS (Required):
                % duration = the simulation duration (in secs)
            % INPUTS (Optional as name-value pairs):
                % 'timestep',timestep = step size (in secs)
                % 'solver','solver' = the ODE solver to use (default ODE45)
            dfltdur = 10;
            dfltTS = dfltdur*1.0e-4;
            dfltSLVR = 'ode45';
            validSLVRs = {'ode1','ode2','ode3','ode4','ode5','ode45','ode23','ode113'};
            validateSLVR = @(x) any(validatestring(x,validSLVRs));
            validScalarPosNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);
            
            p = inputParser;
            addParameter(p,'duration',dfltdur,validScalarPosNum);
            addParameter(p,'timestep',dfltTS,validScalarPosNum);
            addParameter(p,'solver',dfltSLVR,validateSLVR);
            parse(p,varargin{:});
            hobj.duration = p.Results.duration;
            hobj.timestep = p.Results.timestep;
            hobj.solver = p.Results.solver;
        end
        
        % run
        function [time,states] = run(hobj,opts)
            % Runs the simulation. Optional argument is options for the
            % solver.
            if nargin < 1 || isempty(opts)
                opts = odeset('RelTol',1e-5,'AbsTol',1e-7,'Stats','on');
            end
            simTimes = 0:hobj.timestep:hobj.duration;
            for i=1:1:length(hobj.tether.nodes)
                x0(6*i-5) = hobj.tether.nodes(i).x;
                x0(6*i-4) = hobj.tether.nodes(i).y;
                x0(6*i-3) = hobj.tether.nodes(i).z;
                x0(6*i-2) = hobj.tether.nodes(i).u;
                x0(6*i-1) = hobj.tether.nodes(i).v;
                x0(6*i) = hobj.tether.nodes(i).w;
            end
            x0 = x0.';
            switch hobj.solver
                case 'ode1'
                    states = ode1(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode2'
                    states = ode2(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode3'
                    states = ode3(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode4'
                    states = ode4(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode5'
                    states = ode5(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode45'
                    [time,states] = ode45(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0,opts);
                case 'ode23'
                    [time,states] = ode23(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0,opts);
                case 'ode113'
                    [time,states] = ode113(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0,opts);
                otherwise
                    error('simP.solver not recognized.');
            end
        end % end run
        
        function parseInputFile(hobj,fpfn)
            % You must pass in the fully qualifie (full path) file name
            fid = fopen(fpfn);
            while true
                tline = fgetl(fid);            
                if isnumeric(tline)
                    break;
                end
                if isempty(tline) || tline(1)=='%' || tline(1)=='\'
                    continue;
                end
                eval(tline);
            end
            if isfield(simP,'solver')
                hobj.solver = simP.solver;
            end
            if isfield(simP,'timestep')
                hobj.timestep = simP.timestep;
            end
            if isfield(simP,'totaltime')
                hobj.duration = simP.totaltime;
            end
            hobj.tether = tether(tetherP.length,tetherP.mass,tetherP.radius,...
                tetherP.springk,tetherP.dampFac,tetherP.relativeDensity,tetherP.numNodes);
            hobj.environment = environment;
        end
        
    end % methods
end % simulation