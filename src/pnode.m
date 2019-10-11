classdef pnode < handle
    properties (SetAccess = private)
        springk
        dampc
        pathshape
    end
    
    methods
        
        function hobj = pnode(k,c,ps)
            if nargin < 1
                hobj.springk = 1e5;
                hobj.dampc = 0.25*hobj.springk;
                hobj.pathshape = 'eight';
            else
                hobj.springk = k;
                hobj.dampc = c;
                if ischar(ps)
                    hobj.pathshape = ps;
                else
                    error('Path shape must be a character array. See get.position for valid path shapes.');
                end
            end
        end % constructor
        
        function p = getPosition(hobj,t,a,b,om)
            if nargin < 3
                a = 1;
                b = 1;
                om = 1;
            end
            switch hobj.pathshape
                case 'eight'
                    p(1) = a*sin(om*t);
                    p(2) = b/a*p(1)*cos(om*t);
                case 'ellipse'
                    p(1) = a*sin(om*t);
                    p(2) = b*cos(om*t);
                otherwise
                    error('Unknown pathshape.');
            end % switch pathshape
        end % getposition
        
    end % methods
end % pnode