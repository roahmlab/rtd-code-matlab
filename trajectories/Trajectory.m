classdef Trajectory < handle
    % Trajectory
    % This encapsulates the conversion of parameters used in optimization
    % to the actual trajectory generated from those parameters. It's also
    % indirectly used to specify the OptimizationEngine's size
    properties (Constant, Abstract)
        % Size of the trajectoryParams, we want this set for all use
        param_shape {mustBePositive, mustBeInteger}
    end
    properties
        % The parameters used for the trajectory
        trajectoryParams {mustBeNumeric}
        % The t_0 of the trajectory
        startTime {mustBeNumeric, mustBeScalarOrEmpty}
    end
    methods (Abstract)
        % An example constructor for the trajectory object. Should be
        % implemented with varargin
        %self = Trajectory(              ...
        %            robotState,         ...
        %            trajectoryParams    ...
        %        )
        
        % A validated method to set the parameters for the trajectory.
        % Should be implemented with a varargin.
        setTrajectory(                  ...
                    self,               ...
                    trajectoryParams,   ...
                    robotState          ...
                )
        
        % A validated method to get the trajectory parameters.
        % throws RTD:InvalidTrajectory if there isn't a trajectory to get.
        [trajectoryParams, startTime] = getTrajParams(self)
        
        % Computes the actual input commands for the given time.
        % throws RTD:InvalidTrajectory if the trajectory isn't set
        command = getCommand(self, time)
    end
end