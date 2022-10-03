classdef ForwardOccupancy < ReachableSets
    % ArmTdForwardOccupancy
    % This either encapsulates the reachable sets in memory, or enables the
    % online computation of reachable sets. It acts as a generator for a
    % single instance of ReachableSet
    properties
        cache_max_size = 0 % we don't want to cache any forward occupancy instances
        robotInfo
        jrsHandle
        smooth_obs;
    end
    methods
        % An example constructor, but can take anything needed
        function self = ForwardOccupancy( ...
                    robotInfo, jrsHandle, smooth_obs ...
                )
            self.robotInfo = robotInfo;
            self.jrsHandle = jrsHandle;
            self.smooth_obs = smooth_obs;
        end
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns ReachbleSet
        function reachableSet = generateReachableSet(self, robotState)
            % Computes the forward kinematics and occupancy
            
            % First get the JRS (allow the use of a cached value if it
            % exists)
            jrsInstance = self.jrsHandle.getReachableSet(robotState, false);
            
            % get forward kinematics and forward occupancy
            for i = 1:jrsInstance.n_t
               [R_w{i, 1}, p_w{i, 1}] = pzfk(jrsInstance.R{i, 1}, self.robotInfo.params.pz_nominal); 
               for j = 1:self.robotInfo.params.pz_nominal.num_bodies
                  FO{i, 1}{j, 1} = R_w{i, 1}{j, 1}*self.robotInfo.link_poly_zonotopes{j, 1} + p_w{i, 1}{j, 1}; 
                  FO{i, 1}{j, 1} = reduce(FO{i, 1}{j, 1}, 'girard', self.robotInfo.params.pz_interval.zono_order);
                  FO{i, 1}{j, 1} = remove_dependence(FO{i, 1}{j, 1}, jrsInstance.k_id(end));
               end
            end
            
            reachableSet = FOInstance(self.robotInfo, R_w, p_w, FO, jrsInstance, self.smooth_obs);
        end
    end
end