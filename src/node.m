classdef node < handle
    properties (SetAccess = private)
        mass     % Mass of the node
        buoyancy % buoyancy of the node
    end
    properties
        x % X-coordinate
        y % Y-coordinate
        z % Z-coordinate
        u % X-coordinate
        v % Y-coordinate
        w % Z-coordinate
    end
    
    methods
        % Constructor
        function n = node(m,b,x,y,z)
            n.mass = m;
            n.buoyancy = b;
            n.x = x;
            n.y = y;
            n.z = z;
        end
        
        % Other class methods
    end
end