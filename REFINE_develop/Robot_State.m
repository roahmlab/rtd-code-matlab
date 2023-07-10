%sets the Robot State for REFINE

classdef Robot_State < rtd.entity.states.EntityState

    %properties imported --> state,time,uuid
    %additional properties for REFINE --> state contains --> x,y,h,u,v,r

    methods

         function self = Robot_State(mat)
            self.state = struct('x', mat(1,:), 'y', mat(2,:), 'h', mat(3,:), 'u', mat(4,:), 'v', mat(5,:), 'r', mat(6,:));
            self.time = mat(20,:);
         end
    
    end
end