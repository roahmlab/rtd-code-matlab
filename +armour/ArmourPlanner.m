classdef ArmourPlanner < rtd.planner.RtdPlanner
    properties
        %trajOptProps
        %robot
        %worldInfo
        jrsHandle
        foHandle
        irsHandle
        reachableSets
        objective
        optimizationEngine
        trajectoryFactory
        
        trajopt
    end
    % This class specifies the overall planner, so it sets up everything
    % for any special type of rtd.planner.RtdPlanner.
    methods
        % The constructor would do the following
        % - determine RTD_TrajOpt parameters
        % - Construct OptimizationEngine(s) with parameters
        % - Construct ReachableSets for each reachable set and trajectory type
        % - Construct some Objective object for problem at hand
        
        % - Create RTD_TrajOpt for each trajectory type
        % - Create initial trajectories for each trajectory type
        function self = ArmourPlanner( ...
                    trajOptProps, robot, ...
                    input_constraints_flag, use_robust_input, smooth_obs, traj_type ...
                )
            
            % conduct validation here
            % TODO
            
            %trajOptProps = TrajOptProps;
            %trajOptProps.timeout = 0.5;
            %trajOptProps.horizon = 1.0;
            %timeForCost = 0.5;
            
            %robot = ArmRobotInfo;
            %worldInfo = WorldInfo;
            
            %input_constraints_flag = false;
            %use_robust_input = false;
            %smooth_obs = false;
            
            % Create our reachable sets
            rsGenerators = struct;
            rsGenerators.jrs = armour.reachsets.JRSGenerator(robot, "traj_type", traj_type);
            rsGenerators.fo = armour.reachsets.FOGenerator(robot, rsGenerators.jrs, smooth_obs);
            if input_constraints_flag
                rsGenerators.irs = armour.reachsets.IRSGenerator(robot, rsGenerators.jrs, use_robust_input);
            end
            self.reachableSets = rsGenerators;
            
            % Create the trajectoryFactory
            % Compat
            if strcmp(traj_type,'orig')
                traj_type = 'piecewise';
            end
            self.trajectoryFactory = armour.trajectory.ArmTrajectoryFactory(trajOptProps, traj_type);
            
            % Create the objective
            self.objective = rtd.planner.trajopt.GenericArmObjective(trajOptProps, self.trajectoryFactory);
            
            % Selection the optimization engine
            self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
            
            % Create the trajopt object.
            self.trajopt = {rtd.planner.trajopt.RtdTrajOpt( ...
                trajOptProps,           ...
                robot,              ...
                self.reachableSets,          ...
                self.objective,              ...
                self.optimizationEngine,     ...
                self.trajectoryFactory)};
        end
    
        % Then on each waypoint, we call for a trajectory plan:
        % Use RTD to solve for a trajectory and return either
        % the parameters or invalid signal (continue)
        function [trajectory, info] = planTrajectory(self, robotState, worldState, waypoint)
            % Loops over each RTD_TrajOpt instance (thus, each trajectory
            % type) with the given RobotState, rtd.sim.world.WorldState, Waypoint, and
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
            [trajectory, ~, info] = self.trajopt{1}.solveTrajOpt(robotState, worldState, waypoint);
        end
    end
end