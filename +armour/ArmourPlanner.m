classdef ArmourPlanner < rtd.planner.RtdPlanner & rtd.util.mixins.Options
    properties
        rsGenerators
        objective
        optimizationEngine
        trajectoryFactory
        
        trajopt
    end
    
    methods (Static)
        function options = defaultoptions()
            options.input_constraints_flag = false;
            options.use_robust_input = false;
            options.smooth_obs = false;
            options.traj_type = 'piecewise';
            options.verboseLevel = 'DEBUG';
        end
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
        function self = ArmourPlanner(trajOptProps, robot, options)
            arguments
                trajOptProps (1,1) rtd.planner.trajopt.TrajOptProps
                robot (1,1) armour.ArmourAgent
                options.input_constraints_flag (1,1) logical
                options.use_robust_input (1,1) logical
                options.smooth_obs (1,1) logical
                options.traj_type {mustBeMember(options.traj_type,{'piecewise','bernstein'})}
                options.verboseLevel (1,1) rtd.util.types.LogLevel
            end
            options = self.mergeoptions(options);
            
            % Create our reachable sets
            self.rsGenerators = struct;
            self.rsGenerators.jrs = armour.reachsets.JRSGenerator(robot, traj_type=options.traj_type);
            self.rsGenerators.fo = armour.reachsets.FOGenerator(robot, self.rsGenerators.jrs, smooth_obs=options.smooth_obs);
            if options.input_constraints_flag
                self.rsGenerators.irs = armour.reachsets.IRSGenerator(robot, self.rsGenerators.jrs, use_robust_input=options.use_robust_input);
            end
            
            % Create the trajectoryFactory
            self.trajectoryFactory = armour.trajectory.ArmTrajectoryFactory(trajOptProps, options.traj_type);
            
            % Create the objective
            self.objective = rtd.planner.trajopt.GenericArmObjective(trajOptProps, self.trajectoryFactory);
            
            % Selection the optimization engine
            self.optimizationEngine = rtd.planner.trajopt.FminconOptimizationEngine(trajOptProps);
            
            % Create the trajopt object.
            self.trajopt = {rtd.planner.trajopt.RtdTrajOpt( ...
                trajOptProps,           ...
                robot,              ...
                self.rsGenerators,          ...
                self.objective,              ...
                self.optimizationEngine,     ...
                self.trajectoryFactory)};
        end
    
        % Then on each waypoint, we call for a trajectory plan:
        % Use RTD to solve for a trajectory and return either
        % the parameters or invalid signal (continue)
        function [trajectory, info] = planTrajectory(self, robotState, worldState, waypoint)
            arguments
                self armour.ArmourPlanner
                robotState rtd.entity.states.ArmRobotState
                worldState
                waypoint
            end
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