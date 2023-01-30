classdef RtdTrajOpt < handle & rtd.util.mixins.NamedClass
    % RtdTrajOpt
    % This object handles the necessary calls to perform the actual
    % trajectory optimization when requested.
    properties
        trajOptProps rtd.planner.trajopt.TrajOptProps
    end
    % Make protected later (Debate).
    properties%(Access = 'protected')
        robotInfo
        reachableSets
        objective
        optimizationEngine
        trajectoryFactory
    end
    methods
        % Store all the handles to objects that we want to use.
        % This should encapsulate the main trajectory optimization
        % aspection of RTD & ideally need very little specialization
        % between versions.
        function self = RtdTrajOpt(    ...
                    trajOptProps,       ...
                    robotInfo,          ...
                    reachableSets,      ...
                    objective,          ...
                    optimizationEngine, ...
                    trajectoryFactory   ...
                )
            self.trajOptProps = trajOptProps;
            self.robotInfo = robotInfo; % there's a chance these two info
            self.reachableSets = reachableSets;
            self.objective = objective;
            self.optimizationEngine = optimizationEngine;
            self.trajectoryFactory = trajectoryFactory;
        end
        
        % Execute the solver for trajectory optimization.
        % Returns a Trajectory object and an associated cost.
        function [trajectory, cost, info] = solveTrajOpt(  ...
                    self,                   ...
                    robotState,             ...
                    worldState,             ...
                    waypoint,               ...
                    initialGuess            ...
                )

            self.vdisp("Generating reachable sets and nonlinear constraints", 'INFO')
            
            % Generate reachable sets
            rsInstances = struct;
            for rs_name=fieldnames(self.reachableSets).'
                self.vdisp(['Generating ', rs_name{1}], 'DEBUG')
                rs = self.reachableSets.(rs_name{1}).getReachableSet(robotState, false);
                rsInstances.(rs_name{1}) = rs;
            end

            % Generate nonlinear constraints
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
                    new_bounds = repmat(new_bonds, num_parameters, 1);
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
            self.vdisp("Optimizing!")
            [success, parameters, cost] = self.optimizationEngine.performOptimization(guess, ...
                objectiveCallback, constraintCallback, bounds);
            
            % if success
            if success
                trajectory = self.trajectoryFactory.createTrajectory(robotState, rsInstances, parameters);
            else
                trajectory = [];
            end
            
            % Create an info struct for return
            info.worldState = worldState;
            info.robotState = robotState;
            info.rsInstances = rsInstances;
            info.nlconCallbacks = nlconCallbacks;
            info.objectiveCallback = objectiveCallback;
            info.waypoint = waypoint;
            info.bounds = bounds;
            info.n_k = num_parameters;
            info.guess = guess;
            info.trajectory = trajectory;
            info.cost = cost;
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