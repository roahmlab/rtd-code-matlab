classdef FRS_Instance_speed_change < rtd.planner.reachsets.ReachSetInstance & refine.Offline_Reachability_Analysis.vehicle_dynamics_generation
    % Similar to FOInstance
    % This is just an individual instance of FRS speed change
    % generation.
    properties
        input_range = [-1.0, 1.0]
        num_parameters = 0
        
        % properties carried over from the original implementation
        sliceable %info about the slicing, dimension to slice along,x,y,h dimensions --> p_u
        slicedinfo %result of slice operation, sliced values along x,y,h dimension, sum along each dimension
        zonoSliceInfo %o/p of sliced operation & vector of sliceable values
        Vehrs %a single reachable set consisting of xy center points, heading, zono gen x,y,h, no of gen of each zono, zonoslice info for each zono, t and others
        %frsSelectInfo %info of specific frs, maneuver type, index, if frs is mirrored
        frsMega % sets of frs for diff maneuvers and additional info
        frsTotal %total frs and other parameters together
    end

    methods
        function self = FRS_Instance_speed_change( ...
                    sliceable,sliceableinfo,zonoSLiceInfo,Vehrs,frsMega,frsTotal ...
                )
            self.sliceable = sliceable;
            self.slicedinfo = sliceableinfo;
            self.zonoSliceInfo = zonoSLiceInfo;
            self.Vehrs = Vehrs;
            %self.frsSelectInfo = frsSelectInfo;
            self.frsMega = frsMega;
            self.frsTotal = frsTotal;
            
        end
        
        % Handles the obstacle-frs pair or similar to generate the
        % nlconstraint.
        % Returns a function handle for the nlconstraint generated
        % where the function's return type is [c, ceq, gc, gceq]
        function nlconFunction = genNLConstraint(self, worldState)
            
            sliceable = self.sliceable;
            vehrs = self.Vehrs;
            %check if the speed is btw the max and min speed
            speed_constraint = @(x) speedConstraint(x,sliceable,vehrs);%check with z0vel
            
    
        end

        function [c,ceq] = speedConstraint(x,sliceable,vehrs)
        %calculate the speed
        speed = x(1); %needs to be passed
        speed_min = sliceable(1);
        speed_max = sliceable(2);
        
        %check if the speed is between the limit
        if(speed<speed_min)
            c = speed - speed_min; %add c
        elseif (speed>speed_max)
            c = speed_max - speed; %subtract c
        else 
            c=0;
        end
        ceq = []; %equality constraint
        
        end
    end
end 
    

