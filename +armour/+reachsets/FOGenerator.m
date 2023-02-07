classdef FOGenerator < rtd.planner.reachsets.ReachSetGenerator
    % ForwardOccupancy
    % This acts as a generator for a single instance of a
    % ForwardReachableSet, and return FOInstance
    
    % Required Abstract Properties
    properties
        cache_max_size = 0 % We don't want to cache any forward occupancy instances
    end

    % Additional Properties
    properties
        jrsGenerator
        smooth_obs
        robot
    end
    
    methods
        function self = FOGenerator(robot, jrsGenerator, options)
            arguments
                robot armour.ArmourAgent
                jrsGenerator armour.reachsets.JRSGenerator
                options.smooth_obs(1,1) logical = false
                options.verboseLevel(1,1) rtd.util.types.LogLevel = "DEBUG"
            end
            self.robot = robot;
            self.jrsGenerator = jrsGenerator;
            self.smooth_obs = options.smooth_obs;
            self.set_vdisplevel(options.verboseLevel);
        end
    end
    methods (Access=protected)
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns FOInstance
        function reachableSet = generateReachableSet(self, robotState)
            % Computes the forward kinematics and occupancy
            
            % First get the JRS (allow the use of a cached value if it
            % exists)
            jrsInstance = self.jrsGenerator.getReachableSet(robotState, ignore_cache=false);
            
            self.vdisp("Generating forward occupancy!", "INFO")

            % get forward kinematics and forward occupancy
            for i = 1:jrsInstance.n_t
               [R_w{i, 1}, p_w{i, 1}] = armour.legacy.dynamics.pzfk(jrsInstance.R{i, 1}, self.robot.info.params.pz_nominal);
               for j = 1:self.robot.info.params.pz_nominal.num_bodies
                  FO{i, 1}{j, 1} = R_w{i, 1}{j, 1}*self.robot.info.links(j).poly_zonotope + p_w{i, 1}{j, 1}; 
                  FO{i, 1}{j, 1} = reduce(FO{i, 1}{j, 1}, 'girard', self.robot.info.params.pz_interval.zono_order);
                  FO{i, 1}{j, 1} = remove_dependence(FO{i, 1}{j, 1}, jrsInstance.k_id(end));
               end
            end
            
            reachableSet = armour.reachsets.FOInstance(self.robot.info, R_w, p_w, FO, jrsInstance, self.smooth_obs);
        end
    end
end