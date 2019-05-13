classdef link < handle
    % A link is a discrete part of a tether.
    % All link types have these properties, though the value may be zero.
    properties (SetAccess = protected)
        linktype  % The type of link
        baselength   % The unstretched length of the link
        radius       % The radius of the link
        mass         % The mass of the link
        fromnode     % The node at the start of the link
        tonode       % The node at the end of the link
    end
    
    methods
        % Constructor
        function hobj = link(linktype,baselength,radius,mass,fromnode,tonode)
            hobj.linktype = linktype;
            hobj.baselength = baselength;
            hobj.radius = radius;
            hobj.mass = mass;
            hobj.fromnode = fromnode;
            hobj.tonode = tonode;
        end % End Constructor
        
        % Other Methods
        
    end % End Methods
end