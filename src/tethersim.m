classdef tethersim < handle & simulation
    % a tether sim is a specific type of simulation
    properties
        tether   % one or more tether objects
        environment  % the environment that the tether is in
    end
    
    methods
        % Constructor
        function hobj = tethersim(simargs,tether,environment)
            error('tethersim is currently inoperable. use simulation');
            if nargin == 0
                error('Simulation duration is a required argument.');
            end
            hobj@simulation(simargs);
            hobj.tether = tether;
            hobj.environment = environment;
        end
        
        % run
        function run(hobj,opts)
            % Runs the simulation. Optional argument is options for the
            % solver.
            if nargin < 1
                opts = odeset('RelTol',1e-5,'AbsTol',1e-7,'Stats','on');
            end
            simTimes = 0:hobj.timestep:hobj.duration;
            x0 = hobj.tether.statevec;
            switch simP.solver
                case 'ode1'
                    svec = ode1(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode2'
                    svec = ode2(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode3'
                    svec = ode3(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode4'
                    svec = ode4(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode5'
                    svec = ode5(@(t, x)hobj.tether.derivative(t, x, hobj.environment),simTimes,x0);
                    time = simTimes;
                case 'ode45'
                    [time,svec] = ode45(@(t, x)hobj.tether.derivative(t, x, thr),simTimes,x0,opts);
                case 'ode23'
                    [time,svec] = ode23(@(t, x)hobj.tether.derivative(t, x, thr),simTimes,x0,opts);
                case 'ode113'
                    [time,svec] = ode113(@(t, x)hobj.tether.derivative(t, x, thr),simTimes,x0,opts);
                otherwise
                    error('simP.solver not recognized.');
            end
        end % end run
        
    end % end methods
    
end