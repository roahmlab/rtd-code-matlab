classdef GenericArmObjective < rtd.planner.trajopt.Objective
    % GenericArmObjective
    % This objective generalizes the joint space objective used for ArmTD
    % and allows us to use any joint-space trajectory for ArmTD based
    % trajopt ptoblems. It should be setup in the planner itself and the
    % handle for this object passed to the trajopt problem.
    properties
        % we just need to time for the cost and the factory for the
        % objective
        trajectoryFactory
        t_cost
    end
    methods
        % Construct this objective function generator. Provide a trajectory
        % factory object which creates unique trajectory objects with each
        % call and the properties of the overall trajopt problem.
        function self = GenericArmObjective(trajOptProps, trajectoryFactory)
            % we just need to time for the cost and the factory for the
            % objective
            self.trajectoryFactory = trajectoryFactory;
            self.t_cost = trajOptProps.timeForCost;
        end
        
        % Given the information, generate a handle for an objective
        % function with two return values, where
        % function [cost, grad_cost] = objectiveCallback(params)
        function objectiveCallback = genObjective(self, robotState, waypoint, reachableSets)
            %future intention: q_des = waypoint.q_des;
            q_des = waypoint; % Rename just for past readability

            % Create an trajectory instance for whatever generic trajectory
            % factory we were given.
            trajectoryObj = self.trajectoryFactory.createTrajectory(robotState, reachableSets);

            % create and return the function handle.
            objectiveCallback = @(trajectoryParams) ...
                evalTrajectory(trajectoryParams, trajectoryObj, q_des, robotState.time + self.t_cost);
        end
    end
end

% Helper function purely accessible to this class without any class state
% which a handle can be made to to evaluate the trajectory for the cost.
% Should work for any generic arm trajectory in joint space.
function [cost] = evalTrajectory(trajectoryParams, trajectoryObj, q_des, t_cost)
    trajectoryObj.setParameters(trajectoryParams);
    plan = trajectoryObj.getCommand(t_cost);
    cost = sum((plan.q_des - q_des).^2);
end