classdef Objective < handle
    % Objective
    % This will generate some sort of anonymous function for the
    % optimization engine. Any important dynamics or similar calls can take
    % place inside this object, and it gets set up before the TrajOpt
    % solver.
    methods (Abstract)
        % Recommended Constructor. RobotInfo would hold the parameters
        % loaded from the URDF or similar file. This could be used to
        % compute robot dynamics
        % self = Objective(self, robotInfo)
        
        % Given the information, generate a handle for an objective
        % function with two return values, where
        % function [cost, grad_cost] = objectiveCallback(params)
        objectiveCallback = genObjective(self, robotState, waypoint, reachableSets)
            % This takes the RobotState and Waypoint, and reads properties
            % from ReachableSet to greate the cost function.
            % NOTE: We move timeout to be handled by the OptimizationEngine
            % instead.
    end
end