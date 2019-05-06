classdef flow < handle
    % Types of flow:
        % still - fluid is not moving
        % uniform - velocity is constant throughout the flow
        % linearZ - velocity changes linearly in the z-direction
   properties
      type
   end
   methods
      function hobj = flow(type)
         hobj.type = type;
      end
   end
   enumeration
       still (0)
       uniform (1)
       linearZ (2)
   end
end