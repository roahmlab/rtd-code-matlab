classdef RtdTrajOpt < rtd.util.mixins.NamedClass & handle
% Core trajectory optimization routine for RTD
%
% This object handles the necessary calls to perform the actual trajectory
% optimization when requested. It calls the generators for the reachble
% sets and combines all the resulting nonlinear constraints in the end.
%
% Note:
%   This only optimizes for a single `rtd.planner.trajopt.Objective`
%   object. If you want to optimize for more than one objective, either use
%   multiple `RtdTrajOpt` objects, or design a single objective that combines
%   the multiple objectives.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-08-24
% Last Revised: 2023-02-03
%
% See also rtd.planner.RtdPlanner, rtd.planner.trajopt
%
% --- More Info ---
%
    properties
        trajOptProps rtd.planner.trajopt.TrajOptProps %
        reachableSets %
        objective %
        optimizationEngine %
        trajectoryFactory %
    end
    methods
        function self = RtdTrajOpt(     ...
                    trajOptProps,       ...
                    reachableSets,      ...
                    objective,          ...
                    optimizationEngine, ...
                    trajectoryFactory,  ...
                    options             ...
                )
            % Construct the RtdTrajOpt object.
            % 
            % Store all the handles to objects that we want to use.
            % This should encapsulate the main trajectory optimization
            % aspection of RTD & ideally need very little specialization
            % between versions.
            %
            % Arguments:
            %   trajOptProps (rtd.planner.trajopt.TrajOptProps)
            %   robot (rtd.sim.world.WorldEntity)
            %   reachableSets (struct)
            %   objective (rtd.planner.trajopt.Objective)
            %   optimizationEngine (rtd.planner.trajopt.OptimizationEngine)
            %   trajectoryFactory (rtd.planner.trajectory.TrajectoryFactory)
            %   options: Keyword arguments. See below.
            %
            % Keyword Arguments
            %   verboseLevel (rtd.util.types.LogLevel): The log level to use for this object
            %
            arguments
                trajOptProps (1,1) rtd.planner.trajopt.TrajOptProps
                reachableSets (1,1) struct
                objective (1,1) rtd.planner.trajopt.Objective
                optimizationEngine (1,1) rtd.planner.trajopt.OptimizationEngine
                trajectoryFactory (1,1) rtd.planner.trajectory.TrajectoryFactory
                options.verboseLevel (1,1) rtd.util.types.LogLevel = 'DEBUG'
            end
            self.trajOptProps = trajOptProps;
            self.reachableSets = reachableSets;
            self.objective = objective;
            self.optimizationEngine = optimizationEngine;
            self.trajectoryFactory = trajectoryFactory;
            self.set_vdisplevel(options.verboseLevel);
        end
        
        function [trajectory, cost, info] = solveTrajOpt(  ...
                    self,                   ...
                    robotState,             ...
                    worldState,             ...
                    waypoint,               ...
                    initialGuess,           ...
                    rsAdditionalArgs        ...
                )
            % Execute the solver for trajectory optimization.
            %
            % Note:
            %   The returned `info` struct has the following entries:
            %   worldState, robotState, rsInstances, nlconCallbacks,
            %   objectiveCallback, waypoint, bounds, num_parameters, guess,
            %   trajectory, cost.
            %
            % Note:
            %   Additional arguments for a reachable set should be in a
            %   struct form. These are converted to extra Name-Value
            %   arguments for the respective reachable set.
            %
            % Arguments:
            %   robotState: State of the robot.
            %   worldState: Observed state of the world for the reachable sets
            %   waypoint: Waypoint we want to optimize to
            %   initialGuess (rtd.planner.trajectory.Trajectory): Past trajectory to use as an initial guess
            %   rsAdditionalArgs (Optional struct): additional arguments to pass to the reachable sets, by set name
            %
            % Returns:
            %   [trajectory(rtd.planner.trajectory.Trajectory), cost(double), info(struct)]:
            %   `trajectory` is an instance of the trajectory object that
            %   corresponds to the solved trajectory optimization. If it
            %   wasn't successful, this will either be an invalid
            %   trajectory of empty. `cost` is the final cost of the
            %   objective used. `info` is a struct of optimization data.
            %

            % rsAdditionalArgs is optional, so make sure it exists.
            if ~exist('rsAdditionalArgs', 'var')
                rsAdditionalArgs = struct;
            end
            
            % Generate reachable sets
            self.vdisp("Generating reachable sets and nonlinear constraints", 'INFO')
            rsInstances_arr = struct('id', double.empty(), 'rs', struct, 'num_instances', 0);
            for rs_name=fieldnames(self.reachableSets).'
                self.vdisp(['Generating ', rs_name{1}], 'DEBUG')
                rs_args = {};
                if isfield(rsAdditionalArgs, rs_name{1})
                    self.vdisp(['Passing additional arguments to generate ', rs_name{1}], 'GENERAL')
                    rs_args = namedargs2cell(rsAdditionalArgs.(rs_name{1}));
                end
                rs_struct = self.reachableSets.(rs_name{1}).getReachableSet(robotState, rs_args{:}, ignore_cache=false);
                % expand struct into id's
                for idx=1:length(rs_struct)
                    id = rs_struct(idx).id;
                    rsInstances_arr(id).id = id;
                    rsInstances_arr(id).rs.(rs_name{1}) = rs_struct(idx).rs;
                    if isempty(rsInstances_arr(id).num_instances)
                        rsInstances_arr(id).num_instances = 0;
                    end
                    rsInstances_arr(id).num_instances = rsInstances_arr(id).num_instances + 1;
                end
            end

            % Discard empties
            mask = [rsInstances_arr.num_instances] > 0;
            rsInstances_arr = rsInstances_arr(mask);

            % Generate nonlinear constraints
            successes = false(1, length(rsInstances_arr));
            parameters = cell(1, length(rsInstances_arr));
            costs = zeros(1, length(rsInstances_arr)) + Inf;
            for rsInstances_idx=1:length(rsInstances_arr)
                rsInstances = rsInstances_arr(rsInstances_idx).rs;
                id = rsInstances_arr(rsInstances_idx).id;
                self.vdisp(['Solving problem ', num2str(id)], 'INFO')
                self.vdisp("Generating nonlinear constraints", 'DEBUG')
                rsInstances_cell = struct2cell(rsInstances).';
                nlconCallbacks = cellfun(@(rs)rs.genNLConstraint(worldState), ...
                                        rsInstances_cell, ...
                                        UniformOutput = false);

                % Validate rs sizes
                self.vdisp("Validating sizes", 'DEBUG')
                num_parameters = cellfun(@(rs)rs.num_parameters, rsInstances_cell);
                if ~all(num_parameters == num_parameters(1))
                    error("Reachable set parameter sizes don't match!")
                end
                num_parameters = num_parameters(1);
    
                % Compute slack sizes
                num_slack = cellfun(@(rs)size(rs.input_range,1), rsInstances_cell);
                mask_ones = num_slack == 1;
                num_slack(mask_ones) = num_parameters;
                num_slack = num_slack - num_parameters;
                if any(num_slack < 0)
                    error("Reachable sets have invalid `num_parameters` or invalid size on `input_range`.")
                end
                slack_idxs = [0, cumsum(num_slack)];
    
                % Compute bounds
                self.vdisp("Computing bounds", 'DEBUG')
                param_bounds = [ones(num_parameters,1)*-Inf, ones(num_parameters,1)*Inf];
                slack_bounds = zeros(slack_idxs(end), 2);
                for i=1:length(num_slack)
                    new_bounds = rsInstances_cell{i}.input_range;
                    if mask_ones(i)
                        new_bounds = repmat(new_bounds, num_parameters, 1);
                    elseif num_slack(i) > 0
                        % Isolate and add the slack
                        slack_bounds(slack_idxs(i)+1:slack_idxs(i+1),:) = new_bounds(num_parameters+1:end,:);
                        new_bounds = new_bounds(1:num_parameters,:);
                    end
    
                    % Ensure bounds are the intersect of the intervals for the
                    % parameters
                    param_bounds(:, 1) = max(param_bounds(:, 1), new_bounds(:, 1));
                    param_bounds(:, 2) = min(param_bounds(:, 2), new_bounds(:, 2));
                end
                
                % Combine nlconCallbacks
                % TODO update to allow combination with any other constraints
                % needed if wanted
                constraintCallback = @(k) merge_constraints(k, num_parameters, num_slack, nlconCallbacks, fieldnames(self.reachableSets).');
                
                % create bounds (robotInfo and worldInfo come into play here?)
                bounds.param_limits = [param_bounds; slack_bounds];
                bounds.ouput_limits = [];
                
                % Create the objective
                objectiveCallback = self.objective.genObjective(robotState, ...
                    waypoint, rsInstances);
                
                % If initialGuess is none, or invalid, make a zero
                try
                    guess = initialGuess.trajectoryParams;
                catch
                    guess = [];
                end
                
                
                % Optimize
                self.vdisp("Optimizing!",'INFO')
                [success, parameter, cost] = self.optimizationEngine.performOptimization(guess, ...
                    objectiveCallback, constraintCallback, bounds);
                
                % if success
                if success
                    successes(rsInstances_idx) = true;
                    parameters{rsInstances_idx} = parameter;
                    costs(rsInstances_idx) = cost;
                end
            end
            
            % Select the best cost
            masked_costs = costs(successes);
            masked_parameters = parameters(successes);
            [min_cost, min_idx] = min(masked_costs);

            % if success
            if ~isempty(min_cost)

                idxs = 1:length(successes);
                idxs = idxs(successes);
                rsInstances_idx = idxs(min_idx);
                id = rsInstances_arr(rsInstances_idx).id;
                self.vdisp(['Optimal solution found in problem ', num2str(id)],'INFO');
                rsInstances = rsInstances_arr(rsInstances_idx).rs;
                parameter = masked_parameters{min_idx};
                trajectory = self.trajectoryFactory.createTrajectory(robotState, rsInstances, parameter);
            else
                rsInstances_idx = -1;
                trajectory = [];
            end
            
            % Create an info struct for return
            % TODO: Update for multi problem based system
            info.worldState = worldState;
            info.robotState = robotState;
            info.rsInstances = rsInstances_arr;
            info.nlconCallbacks = nlconCallbacks;
            info.objectiveCallback = objectiveCallback;
            info.waypoint = waypoint;
            info.bounds = bounds;
            info.num_parameters = num_parameters;
            info.guess = guess;
            info.trajectory = trajectory;
            info.cost = cost;
            info.parameters = parameters;
            info.successes = successes;
            info.solution_idx = rsInstances_idx;
        end
    end
end

% Utility function to merge the constraints. nlconNames is added to ease
% debugging if needed.
function [h, heq, grad_h, grad_heq] = merge_constraints(k, num_parameters, num_slack, nlconCallbacks, nlconNames)
    h = [];
    heq = [];
    grad_h = [];
    grad_heq = [];

    slack_idxs = [0, cumsum(num_slack)];
    full_param_size = num_parameters + slack_idxs(end);
    
    for i=1:length(nlconCallbacks)
        if isempty(nlconCallbacks{i})
            continue
        end
        % Run the nlcon callback with the expected parameters
        k_idx = 1:num_parameters;
        if num_slack(i) > 0
            k_idx = [k_idx, slack_idxs(i)+1:slack_idxs(i+1)];
        end
        [rs_h, rs_heq, rs_grad_h, rs_grad_heq] = nlconCallbacks{i}(k(k_idx));

        % Combine
        temp_grad_h = zeros(full_param_size, length(rs_h));
        temp_grad_heq = zeros(full_param_size, length(rs_heq));
        temp_grad_h(k_idx,:) = rs_grad_h;
        temp_grad_heq(k_idx,:) = rs_grad_heq;

        h = [h; rs_h];
        heq = [heq; rs_heq];
        grad_h = [grad_h, temp_grad_h];
        grad_heq = [grad_heq, temp_grad_heq];
    end
end
