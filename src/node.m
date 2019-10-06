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
            n.u = 0;
            n.v = 0;
            n.w = 0;
        end
        
        % Other class methods
        function setNodeMass(hobj,mass)
            hobj.mass = mass;
        end
    end
end