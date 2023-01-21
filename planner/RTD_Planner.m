classdef RTD_Planner < handle
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
    
        % Then on each waypoint, we call for a trajectory plan:
        % Use RTD to solve for a trajectory and return either
        % the parameters or invalid signal (continue)
        trajectory = planTrajectory(self, robotState, worldState, waypoint)
            % Loops over each RTD_TrajOpt instance (thus, each trajectory
            % type) with the given RobotState, rtd.core.world.WorldState, Waypoint, and
            % initial guess

            % From the results, selects the best valid Trajectory,
            % otherwise return an invalid trajectory which will throw when
            % attempting to set the new trajectory, ensuring the old one
            % continues.
    end
end