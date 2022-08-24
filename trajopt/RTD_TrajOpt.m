classdef RTD_TrajOpt < handle
    % This object handles the necessary calls to perform the actual
    % trajectory optimization when requested.
    properties
        timeout
        horizon
    end
    properties(Access = 'protected')
        robotInfo
        worldInfo
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
                    timeout,            ...
                    horizon,            ...
                    robotInfo,          ...
                    worldInfo,          ...
                    reachableSets,      ...
                    objective,          ...
                    optimizationEngine, ...
                    trajectoryFactory   ...
                )
            self.timeout = timeout;
            self.horizon = horizon;
            self.robotInfo = robotInfo; % there's a chance these two info
            self.worldInfo = worldInfo; % structures won't be needed
            self.reachableSets = reachableSets;
            self.objective = objective;
            self.optimizationEngine = optimizationEngine;
            self.trajectoryFactory = trajectoryFactory;
        end
        
        % Execute the solver for trajectory optimization.
        % Returns a Trajectory object and an associated cost.
        function [trajectory, cost] = solveTrajOpt(  ...
                    self,                   ...
                    robotState,             ...
                    worldState,             ...
                    waypoint,               ...
                    initialGuess            ...
                )
            % To implement, below is an outline
            
            % shown is just one, but for all reachablesets.
            rs = self.reachableSets.getReachableSet(robotState);
            nlconCallback = rs.genNLConstraint(worldState);
            
            % Combine nlconCallback with any other constraints needed
            constraintCallback = nlconCallback;
            
            % create bounds (robotInfo and worldInfo come into play here?)
            bounds.param_limits = rs.parameter_limits;
            bounds.ouput_limits = rs.output_limits;
            
            % Create the objective
            objectiveCallback = self.objective.genObjective(robotState, ...
                waypoint, self.reachableSets);
            
            % If initialGuess is none, or invalid, make a zero
            try
                [guess,~] = initialGuess.getTrajParams();
            catch
                guess = zeros(initialGuess.num_params);
            end
            
            % Optimize
            success, parameters, cost = self.optimizationEngine(guess, ...
                objectiveCallback, constraintCallback, bounds, ...
                self.timeout);
            
            % if success
            trajectory = self.trajectoryFactory(robotState, parameters);
        end
    end
end