classdef environment < handle
    
    properties (SetAccess = private)
        fluid
        gravity
    end
    
    methods
        function hobj = environment(varargin)
            % todo add name-value functionality and validate everything on
            % the way in
            hobj.fluid = fluid;
            hobj.gravity = 9.81;
        end
        
        function setGravity(hobj,g)
            hobj.gravity = g;
        end
        
        function setFluid(hobj,f)
            if ~isa(f,'fluid')
                error('You can only set the environment fluid using a fluid object');
            end
            hobj.fluid = f;
        end
    end % methods
end % environment