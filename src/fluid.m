classdef fluid < handle
    % Fluid medium
    % Construct with no arguments for water at 23degC and ~103kPa
    properties (SetAccess = private)
        density     % Fluid density (kg/m^3)
        dynVisc     % Dynamic viscosity (Pa*s)
        kinVisc     % Kinematic viscocity (m^2/s)
        temp        % Temperature (degC)
        pressure    % Pressure (Pa)        
        depth       % Depth of the fluid (m) % todo(rodney) we need another class | maybe ocean < fluid?
    end
    
    properties
        meanvelocity  % 3x1 The mean fluid velocity in the inertial frame
        pointvelocity % 3x1 The mean velocity at a point in the fluid
        flowtype      % Type of flow (todo this needs to be a class. Maybe enums.) 
    end
    
    methods
        function hobj = fluid(density,dynVisc,kinVisc,temp,pressure,meanvelocity,flowtype,depth)
            if nargin < 1 % Make a water object
                hobj.density = 997;
                hobj.dynVisc = 9.482*10^-4;
                hobj.kinVisc = 9.504*10^-7;
                hobj.temp = 23;
                hobj.pressure = 103421;
                hobj.meanvelocity = [0;0;0];
                hobj.flowtype = flow.still;
                hobj.depth = 100; % 100 meters deep. Used for gradient flow
            else
                hobj.density = density;
                hobj.dynVisc = dynVisc;
                hobj.kinVisc = kinVisc;
                hobj.temp = temp;
                hobj.pressure = pressure;
                hobj.meanvelocity = meanvelocity;
                hobj.flowtype = flow(flowtype);
                hobj.depth = depth;
            end
        end
        
        % Other class methods
        function rampvelocity(hobj,t)
            % ramps mean velocity as a function of time
            hobj.meanvelocity = [0.6/(1+exp(-2.0*(t-4)));0;0]; %todo(rodney) parameterize this
        end
        
        function computePointVelocity(hobj,loc)
            % computes the velocity at a point given by x,y,z
            switch hobj.flowtype
                case flow.still
                    v = [0;0;0];
                case flow.uniform
                    % math
                    v = [NaN;NaN;NaN];
                case flow.linearZ
                    % for now assume floor is zero and ramp to some
                    % fraction of depth
                    fracdpth = 0.5;                    
                    if loc(3) > fracdpth*hobj.depth
                        v = hobj.meanvelocity;
                    elseif loc(3) < 0
                        v = [0;0;0];
                    else
                        v = hobj.meanvelocity*loc(3)/(fracdpth*hobj.depth);
                    end
                otherwise
                    error('Unknown flowtype in computePointVelocity');
            end
            hobj.pointvelocity = v;
        end
        
        % setters
        function set.meanvelocity(hobj,v)
            if numel(v) ~= 3
                error('fluid: Mean velocity must be 3x1 vector');
            end
            [m,~] = size(v);
            if m < 3
                v = v.';
            end
            hobj.meanvelocity = v;
        end
        
        function set.flowtype(hobj,flowobj)
            assert(isa(flowobj,'flow'),'flowtype must be a type of flow. Help flow for available types.');
            hobj.flowtype = flowobj;
        end
        
        % getters
        
    end % methods

end % fluid

