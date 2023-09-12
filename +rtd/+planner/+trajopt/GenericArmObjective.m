classdef GenericArmObjective < rtd.planner.trajopt.Objective
% A general configuration-space objective object for arm robots.
%
% This objective generalizes the joint space objective used for ArmTD and
% allows us to use any joint-space trajectory for ArmTD based trajopt
% ptoblems. It should be setup in the planner itself and the handle for
% this object passed to the trajopt problem.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-09-08
% Last revised: 2023-02-03
%
% See also rtd.planner.trajopt, rtd.planner.trajopt.Objective
%
% --- More Info ---
%

    properties
        % Handle to a trajectory factory to get the trajectory to optimize
        trajectoryFactory %

        % The time at which to compare waypoint to.
        t_cost %
    end
    methods
        function self = GenericArmObjective(trajOptProps, trajectoryFactory)
            % Constructs this objective function generator.
            % 
            % Provide a trajectory factory object which creates unique
            % trajectory objects with each call and the properties of the
            % overall trajopt problem, from which the time for the cost is
            % used with some trajectory to create the objective.
            %
            % Arguments:
            %   trajOptProps (rtd.planner.trajopt.TrajOptProps)
            %   trajectoryFactory (rtd.trajectory.TrajectoryFactory)
            %
            arguments
                trajOptProps rtd.planner.trajopt.TrajOptProps
                trajectoryFactory rtd.trajectory.TrajectoryFactory
            end
            self.trajectoryFactory = trajectoryFactory;
            self.t_cost = trajOptProps.timeForCost;
        end
        
        function objectiveCallback = genObjective(self, robotState, waypoint, reachableSets)
            % Given the information, generate a handle for an objective function
            %
            % This creates a function_handle with two return values, where
            % `function [cost, grad_cost] = objectiveCallback(params)`
            %
            % Arguments:
            %   robotState: The starting state of the robot for this optimization
            %   waypoint: The waypoint we want to optimize towards
            %   reachableSets: Instance of the reachable sets used for the objective.
            %
            % Returns:
            %   function_handle: a function handle to be use for the objective callback of the optimizer.
            %

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
    cost = sum((plan.position - q_des).^2);
end