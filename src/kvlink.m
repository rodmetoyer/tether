classdef kvlink < handle & link
    properties (SetAccess = private)
        stiffness % The spring stiffness
        damping   % The damping coefficient (not the damping factor)
    end
    properties
        length
    end
    
    methods
        % Constructor
        function hkv = kvlink(s,d,baselength,radius,mass)
            if nargin < 1
                error('A kvlink must be instantiated with the arguments. There are no default values.');
            end
            hkv@link('kvlink',baselength,radius,mass);
            hkv.length = baselength;
            hkv.stiffness = s;
            hkv.damping = d;
        end
        
        % Other Methods
        
    end
end