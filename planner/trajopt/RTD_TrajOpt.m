classdef RTD_TrajOpt < handle & NamedClass
    % RTD_TrajOpt
    % This object handles the necessary calls to perform the actual
    % trajectory optimization when requested.
    properties
        trajOptProps TrajOptProps
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
        function self = RTD_TrajOpt(    ...
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
            % Generate reachable sets and their relevant constraints.
            self.vdisp("Generating RS's and constraints!")
            rsInstances = {};
            nlconCallbacks = {};
            n_k = 0;
            param_bounds = [];
            for i=1:length(self.reachableSets)
                rs = self.reachableSets{i}.getReachableSet(robotState, false);
                new_n_k = max(n_k, rs.n_k);
                comp = min(n_k, rs.n_k);
                % compute new bounds
                % TODO: consider multiple RS's with slack variables
                new_bounds = zeros(n_k, 2);
                new_bounds(1:n_k,:) = param_bounds;
                new_bounds(1:rs.n_k,:) = rs.parameter_range;
                if comp > 0
                    new_bounds(1:comp, 1) = max(param_bounds(1:comp, 1), new_bounds(1:comp, 1));
                    new_bounds(1:comp, 2) = min(param_bounds(1:comp, 2), new_bounds(1:comp, 2));
                end
                n_k = new_n_k;
                param_bounds = new_bounds;
                % store instances and callbacks
                rsInstances = [rsInstances, {rs}];
                nlconCallbacks = [nlconCallbacks, {rs.genNLConstraint(worldState)}];
            end
            
            % Combine nlconCallback with any other constraints needed
            constraintCallback = @(k) merge_constraints(k, n_k, rsInstances, nlconCallbacks);
            
            % create bounds (robotInfo and worldInfo come into play here?)
            bounds.param_limits = param_bounds;
            bounds.ouput_limits = [];
            
            % Create the objective
            objectiveCallback = self.objective.genObjective(robotState, ...
                waypoint, rsInstances);
            
            % If initialGuess is none, or invalid, make a zero
            if exist('initialGuess','var')
                try
                    [guess,~] = initialGuess.getTrajParams();
                catch
                    guess = zeros(initialGuess.num_params);
                end
            else
                guess = [];
            end
            
            
            % Optimize
            self.vdisp("Optimizing!")
            [success, parameters, cost] = self.optimizationEngine.performOptimization(guess, ...
                objectiveCallback, constraintCallback, bounds);
            
            % if success
            if success
                trajectory = self.trajectoryFactory(robotState, rsInstances, parameters);
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
            info.n_k = n_k;
            info.guess = guess;
            info.trajectory = trajectory;
            info.cost = cost;
        end
    end
end

% Utility function to merge the constraints.
function [h, heq, grad_h, grad_heq] = merge_constraints(k, n_k, rsInstances, nlconCallbacks)
    h = [];
    heq = [];
    grad_h = [];
    grad_heq = [];
    
    for i=1:length(nlconCallbacks)
        temp_k = rsInstances{i}.n_k;
        [rs_h, rs_heq, rs_grad_h, rs_grad_heq] = nlconCallbacks{i}(k(1:temp_k));
        h = [h; rs_h];
        heq = [heq; rs_heq];
        grad_h = [grad_h, [rs_grad_h;zeros(n_k-temp_k, length(rs_h))]];
        grad_heq = [grad_heq, [rs_grad_heq;zeros(n_k-temp_k, length(rs_heq))]];
    end
end