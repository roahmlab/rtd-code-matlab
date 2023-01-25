classdef Trajectory < handle
    % Trajectory
    % This encapsulates the conversion of parameters used in optimization
    % to the actual trajectory generated from those parameters. It's also
    % indirectly used to specify the OptimizationEngine's size
    properties (Constant, Abstract)
        % Size of the trajectoryParams, we want this set for all use
        param_shape {mustBePositive, mustBeInteger}
        % TODO - update to add dynamic parameter as an option
    end
    properties
        % Properties from the trajectory optimization, which also describe
        % properties for the trajectory.
        trajOptProps rtd.planner.trajopt.TrajOptProps
        % The parameters used for the trajectory
        trajectoryParams {mustBeNumeric}
        % The initial state for the trajectory
        startState
    end
    methods (Abstract)
        % An example constructor for the trajectory object. Should be
        % implemented with varargin for cross compatibility.
        %self = Trajectory(              ...
        %            trajOptProps,       ...
        %            robotState,         ...
        %            rsInstances,        ...
        %            trajectoryParams,   ...
        %            varargin            ...
        %        )
        
        % A validation method to ensure that the trajectory this object
        % describes is fully set. Add an additional argument to allow
        % throwing an error if incorrect.
        valid = validate(self, throwOnError)

        % Set the parameters for the trajectory. This allows for the entire
        % trajectory described to be changed, but it should focus on this
        % trajectory params while the constructor should focus on the start
        % state.
        % Should be implemented with a varargin for compatability with all
        % classes (some may want more parameters.)
        setTrajectory(                  ...
                    self,               ...
                    trajectoryParams,   ...
                    rsInstances,        ...
                    robotState,         ...
                    varargin            ...
                )
        

        % Computes the actual input commands for the given time.
        % Should throw RTD:InvalidTrajectory if the trajectory isn't set
        command = getCommand(self, time)
    end
    methods
        % A method to get the trajectory parameters which also validates
        % the trajectory.
        % throws RTD:InvalidTrajectory if there isn't a valid trajectory.
        function [trajectoryParams, startState] = getTrajParams(self)
            % Validate, if invalid, throw
            self.validate(true);
            % Return values
            trajectoryParams = self.trajectoryParams;
            startState = self.startState;
        end
    end
end