classdef tether < handle
    % The tether class represents a one-dimensional object comprised of
    % links and nodes.
    
    % Generally-constant properties of a tether
    properties (SetAccess = protected)
        nodes   % array of node objects
        links   % array of link objects
        length  % total length of the tether
        mass    % total mass of the tether
        radius  % mean radius of the tether
        spring  % total spring constant of the tether
        damp    % damping factor of the tether
        reldens % density relative to water (i.e. 1000 kg/m^3)
    end
    % Variable properties of the tether
    properties
        time     % Current tether time (not necessarily global time)
        endforce % 3x1 force at the end of the tether (todo FRAME?)
    end
    
    methods
        function hobj = tether(length,mass,radius,spring,damp,reldens,n)
            % tether constructor
            % todo(rodney) implement heterogeneous discretization. Also,
            % add a model type arg (KV,STD, and bar to start) and make a
            % tether props class for the properties maybe.
            % For now, discretization is homogenous such that all args are
            % 1x1 scalars and element properties are even distributions of
            % the tether properties.
            % in the future I'll add functionality to instantiate a
            % heterogeneous tether using two arrays (1 links 1 nodes).
            if nargin > 2
                hobj.nodes = node.empty(0,n);
                hobj.links = kvlink.empty(0,n-1); % todo(rodney) need to add functionality for any link
                hobj.nodes(1) = node(mass/n,mass/n*9.81*(1/reldens),0,0,0); % First node at origin of inertial frame
                for i=2:1:n
                    % Create nodes
                    hobj.nodes(i) = node(mass/n,mass/n*9.81*(1/reldens),length/(n-1)*(i-1),0,0); % Initialize pointing in the x direction so nodes are not stacked if user doesn't adjust.
                    % Create links. A link is created forward of the node until you reach the last node.
                    hobj.links(i-1) = kvlink(n*spring,2*damp*n*sqrt(mass*spring)/(n-1),length/(n-1),radius,0,hobj.nodes(i-1),hobj.nodes(i)); 
                end
                hobj.length = length;
                hobj.mass = mass;
                hobj.radius = radius;
                hobj.spring = spring;
                hobj.damp = damp;
                hobj.time = 0;
                hobj.reldens = reldens;
            else % Create a tether from an array of links
                % todo(rodney) add the functionality
                error('Tether assembly from links and nodes is a future feature and not currently supported');
            end
        end
        
        % SetNodeStates
        function setNodeStates(hobj,pos,spd)
            % setNodeStates(hobj,pos,spd)
            % Note: does not change link properties
            % pos is a 3xnubNodes vector of node positions
            % spd is a 3xnubNodes vector of node speeds
            
            % Make sure state vecs are same size as number nodes
            [~,temp1] = size(pos);
            [~,temp2] = size(spd);
            if numel(hobj.nodes) ~= temp1 || numel(hobj.nodes) ~= temp2
                error('Position and speed vectors must be same size as the number of nodes');
            end
            % Loop over nodes and set position and speeds
            for i=1:1:numel(hobj.nodes)
                hobj.nodes(i).x=pos(1,i);
                hobj.nodes(i).y=pos(2,i);
                hobj.nodes(i).z=pos(3,i);
                hobj.nodes(i).u=spd(1,i);
                hobj.nodes(i).v=spd(2,i);
                hobj.nodes(i).w=spd(3,i);
            end                
        end
        
        function xdot = derivative(hobj,t,x,env)
            % returns the derivative of the tether state vector
            numnodes = numel(hobj.nodes);    % number of nodes
            xdot = NaN(6*numnodes,1);       % preallocate state derivative vector
            gravity = 9.81;                 % todo make this come from the enviroment object.

            % Update first 3*numnodes elements in xdot
            xdot(1:3*numnodes) = x(3*numnodes+1:end);

            % First node
            % The first node is geometrically constrained
            r = [x(4);x(5);x(6)]-[x(1);x(2);x(3)];
            lr = norm(r);
            unitr = r/lr;
            stretch = lr-hobj.links(1).baselength;
            flink = [0;0;0];
            v = [x(3*numnodes+4);x(3*numnodes+5);x(3*numnodes+6)] - [x(3*numnodes+1);x(3*numnodes+2);x(3*numnodes+3)];
            stretchd = (r(1)*v(1)+r(2)*v(2)+r(3)*v(3))/lr;
            if stretch > 0
                flink = (stretch*hobj.links(1).stiffness + stretchd*hobj.links(1).damping)*unitr;
            end
            % Add buoyancy
            % ftot = flink + [0;0;hobj.nodes(1).buoyancy];

            % Add drag - none needed for the current implementation, but in general the
            % first node may not be constrained. Need to fgure out if I am going to
            % make that a law or not once I get the implementation ironed out.

            % Update acceleration (remember gravity) - first node constrained
            xdot(3*numnodes+1) = 0; % ftot(1)/hobj.nodes(1).mass;
            xdot(3*numnodes+2) = 0; % ftot(2)/hobj.nodes(1).mass;
            xdot(3*numnodes+3) = 0; % ftot(3)/hobj.nodes(1).mass-gravity;

            % Internal nodes
            fprevlink = flink; % force on the previous node
            for i=2:1:numnodes-1
                r = [x(3*i+1);x(3*i+2);x(3*i+3)]-[x(3*i-2);x(3*i-1);x(3*i)];
                lr = norm(r);
                unitr = r/lr;
                stretch = lr-hobj.links(i).baselength;
                flink = [0;0;0];
                v = [x(3*numnodes+3*i+1);x(3*numnodes+3*i+2);x(3*numnodes+3*i+3)] - [x(3*numnodes+3*i-2);x(3*numnodes+3*i-1);x(3*numnodes+3*i)];
                stretchd = (r(1)*v(1)+r(2)*v(2)+r(3)*v(3))/lr;
                if stretch > 0
                    flink = (stretch*hobj.links(i).stiffness + stretchd*hobj.links(i).damping)*unitr;
                end

                % Add buoyancy and previous link
                ftot = flink + [0;0;hobj.nodes(i).buoyancy] - fprevlink;

                % Add drag

                % Update acceleration (remember gravity) - first node constrained
                xdot(3*numnodes+3*i-2) = ftot(1)/hobj.nodes(i).mass;
                xdot(3*numnodes+3*i-1) = ftot(2)/hobj.nodes(i).mass;
                xdot(3*numnodes+3*i) = ftot(3)/hobj.nodes(i).mass-gravity;
                fprevlink = flink; % update force on the node from the previous line
            end

            % Add buoyancy, previous link, end force for End node
            hobj.endforce = [0;0;0]; % todo where does the end force come from? Passed in?
            ftot = [0;0;hobj.nodes(numnodes).buoyancy] - fprevlink + hobj.endforce;

            % Add drag end node

            % Update acceleration (remember gravity) - first node constrained
            xdot(3*numnodes+3*numnodes-2) = ftot(1)/hobj.nodes(numnodes).mass;
            xdot(3*numnodes+3*numnodes-1) = ftot(2)/hobj.nodes(numnodes).mass;
            xdot(3*numnodes+3*numnodes) = ftot(3)/hobj.nodes(numnodes).mass-gravity;

        end % end derivative
        
        % Adds links and nodes
        function addLink(hobj,node1,node2)
            % Make a new link between the two nodes
            error('addLink not functional');
            % Add the link to the list
        end
        
        function addNode(hobj,node1,node2)
            % Make a new link between the two nodes
            error('addNode not functional');
            % Add the link to the list
        end
        
        % Advance tether time
        function advanceTime(hobj,dt)
            % Advance the states from time to time+dt
            hobj.time = hobj.time+dt;
        end
    end
end