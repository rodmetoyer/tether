classdef fluid < handle
    % Fluid medium
    % Construct with no arguments for water at 23degC and ~103kPa
    properties (SetAccess = private)
        density     % Fluid density (kg/m^3)
        dynVisc     % Dynamic viscosity (Pa*s)
        kinVisc     % Kinematic viscocity (m^2/s)
        temp        % Temperature (degC)
        pressure    % Pressure (Pa)        
    end
    
    properties
        meanvelocity % 3x1 The mean fluid velocity in the inertial frame
        flowtype     % Type of flow (todo this needs to be a class)
        qwrypnts     % 3xn Vector of fluid qwery points. 
    end
    properties (Dependent)
        pointvelocity    % 3xn The fluid velocity at a point(s) in the inertial frame - depends on qwrypnts and calculated based on flowtype
    end
    
    methods
        function hobj = fluid(density,dynVisc,kinVisc,temp,pressure,meanvelocity)
            if nargin < 1 % Make a water object
                hobj.density = 997;
                hobj.dynVisc = 9.482*10^-4;
                hobj.kinVisc = 9.504*10^-7;
                hobj.temp = 23;
                hobj.pressure = 103421;
                hobj.meanvelocity = [0;0;0];
                hobj.flowtype = uniform;
            else
                hobj.density = density;
                hobj.dynVisc = dynVisc;
                hobj.kinVisc = kinVisc;
                hobj.temp = temp;
                hobj.pressure = pressure;
                hobj.meanvelocity = meanvelocity;
            end
        end
        
        % Other class methods
        function rampvelocity(hobj,t)
            hobj.meanvelocity = [0.6/(1+exp(-2.0*(t-4)));0;0]; %todo(rodney) parameterize this
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
        
        function set.flowtype(hobj,t)
            if t < uniform || t > custom
                error('Only 3 flowtypes currently supported. Check enumerations.');
            else
                hobj.flowtype = t;
            end
        end
        
        % getters
    end
    enumeration
        uniform (1) % Uniform is always 1
        zgradient (2)
        % Add additional enums here as development continues.
        % Remember to change custom to one more than what you added.
        custom (3)  % Custom is always the last one
    end
end

