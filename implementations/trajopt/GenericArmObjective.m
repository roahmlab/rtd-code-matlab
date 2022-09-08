classdef GenericArmObjective < Objective
    % Objective
    % This will generate some sort of anonymous function for the
    % optimization engine. Any important dynamics or similar calls can take
    % place inside this object, and it gets set up before the TrajOpt
    % solver.
    properties
        trajectoryFactory
        t_cost
    end
    methods
        % Recommended Constructor. RobotInfo would hold the parameters
        % loaded from the URDF or similar file. This could be used to
        % compute robot dynamics
        function self = GenericArmObjective(trajectoryFactory, timeForCost)
            self.trajectoryFactory = trajectoryFactory;
            self.t_cost = timeForCost;
        end
        
        % Given the information, generate a handle for an objective
        % function with two return values, where
        % function [cost, grad_cost] = objectiveCallback(params)
        function objectiveCallback = genObjective(self, robotState, waypoint, reachableSets)
            %idea: q_des = waypoint.q_des;
            q_des = waypoint; % Rename just for past readability
            trajectoryObj = self.trajectoryFactory(robotState, reachableSets);
            objectiveCallback = @(trajectoryParams) ...
                evalTrajectory(trajectoryParams, trajectoryObj, q_des, self.t_cost);
            
            % This takes the RobotState and Waypoint, and reads properties
            % from ReachableSet to greate the cost function.
            % NOTE: We move timeout to be handled by the OptimizationEngine
            % instead.
        end
    end
end

function [cost] = evalTrajectory(trajectoryParams, trajectoryObj, q_des, t_cost)
    trajectoryObj.setTrajectory(trajectoryParams);
    plan = getCommand(t_cost);
    cost = sum((plan.q_des - q_des).^2);
end