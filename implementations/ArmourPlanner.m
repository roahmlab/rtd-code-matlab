classdef ArmourPlanner < RTD_Planner
    properties
        %trajOptProps
        %robotInfo
        %worldInfo
        
        trajopt
    end
    % This class specifies the overall planner, so it sets up everything
    % for any special type of RTD_Planner.
    methods
        % The constructor would do the following
        % - determine RTD_TrajOpt parameters
        % - Construct OptimizationEngine(s) with parameters
        % - Construct ReachableSets for each reachable set and trajectory type
        % - Construct some Objective object for problem at hand
        
        % - Create RTD_TrajOpt for each trajectory type
        % - Create initial trajectories for each trajectory type
        function self = ArmourPlanner( ...
                    u_ub, u_lb, jrsInstance ...
                )
            
            trajOptProps = TrajOptProps;
            trajOptProps.timeout = 0.5;
            trajOptProps.horizon = 1.0;
            timeForCost = 0.5;
            
            robotInfo = ArmRobotInfo;
            worldInfo = WorldInfo;
            
            input_constraints_flag = false;
            use_robust_input = false;
            smooth_obs = false;
            
            % Create our reachable sets
            jrsHandle = JointReachableSetsOnline(robotInfo);
            foHandle = ForwardOccupancy(robotInfo, jrsHandle, smooth_obs);
            reachableSets = {jrsHandle, foHandle};
            if input_constraints_flag
                irsHandle = InputReachableSet(robotInfo, jrsHandle, use_robust_input);
                reachableSets = [reachableSets, irsHandle];
            end
            
            % Create the trajectoryFactory
            trajectoryFactory = ...
                @(robotState, rsInstances, trajectoryParams)...
                ArmourBernsteinTrajectory(...
                    trajOptProps,       ...
                    robotState,         ...
                    rsInstances,        ...
                    trajectoryParams);
            
            % Create the objective
            objective = GenericArmObjective(trajectoryFactory, timeForCost);
            
            % Selection the optimization engine
            optimizationEngine = FminconOptimizationEngine(trajOptProps,true);
            
            % Create the trajopt object.
            self.trajopt = {RTD_TrajOpt( ...
                trajOptProps,           ...
                robotInfo,              ...
                worldInfo,              ...
                reachableSets,          ...
                objective,              ...
                optimizationEngine,     ...
                trajectoryFactory)};
        end
    
        % Then on each waypoint, we call for a trajectory plan:
        % Use RTD to solve for a trajectory and return either
        % the parameters or invalid signal (continue)
        function trajectory = planTrajectory(self, robotState, worldState, waypoint)
            % Loops over each RTD_TrajOpt instance (thus, each trajectory
            % type) with the given RobotState, WorldState, Waypoint, and
            % initial guess

            % From the results, selects the best valid Trajectory,
            % otherwise return an invalid trajectory which will throw when
            % attempting to set the new trajectory, ensuring the old one
            % continues.
            
            % Later
            %f = parfeval(backgroundPool, ...
            %        @self.trajopt.solveTrajOpt,...
            %        3, ...
            %        robotState, worldState, waypoint);
            %wait(f);
            %[trajectory, ~] = fetchOutputs(f);
            [trajectory, ~] = self.trajopt{1}.solveTrajOpt(robotState, worldState, waypoint);
        end
    end
end