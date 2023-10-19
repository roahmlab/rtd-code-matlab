classdef Trajectory < handle
% Base class for a parameterized trajectory.
%
% This encapsulates the conversion of parameters used in optimization to
% the actual trajectory generated from those parameters. This can also be
% used by an :mat:class:`+rtd.+planner.+trajopt.Objective` object as part
% of the objective function call. It should be generated with some
% :mat:class:`+rtd.+trajectory.TrajectoryFactory`.
%
% Note:
%   The functions `validate(self, throwOnError)`, `setParameters(self,
%   trajectoryParams, options)`, and `getCommand(self, time)` should all be
%   implemented. Check help to find more information.
%
% Note:
%   Many of these functions should be implemented with argument validation
%   to either provide default values or provide additional Name-Value
%   arguments more easily.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-08-24
% Last Revised: 2023-02-02
% 
% See also validate, setParameters, getCommand,
% rtd.trajectory.TrajectoryFactory, arguments,
% rtd.trajectory.TrajectoryContainer
%
% --- Revision History ---
% 2023-09-07 Removed the start state and trajoptprops properties and
%   added the startTime property to reduce class interdependencies.
% 2023-02-02 Added vectorized property
%
% --- More Info ---
%

    properties
        % The parameters used for this trajectory
        trajectoryParams(:,1) double %

        % The time at which this trajectory is valid
        startTime(1,1) double
    end
    
    properties (Abstract, Constant)
        % Set to true if this trajectory supports getting commands for a
        % time vector instead of just a single moment in time.
        vectorized(1,1) logical
    end

    % Expected interfaces for subclasses
    methods (Abstract)
        % Validates if the trajectory is parameterized right.
        %
        % A validation method to ensure that the trajectory this object
        % describes is fully set. Has an additional argument to allow
        % throwing an error if incorrect.
        %
        % Arguments:
        %   throwOnError (logical): whether or not to throw an error if invalid
        %
        % Returns:
        %   logical: whether or not the trajectory is valid
        %
        valid = validate(self, throwOnError)

        % Set the parameters for the trajectory.
        % 
        % This allows for the entire trajectory described to be changed,
        % but it should focus on this trajectory params while the
        % constructor should focus on the start state.
        %
        % Arguments:
        %   trajectoryParams: the parameters of the trajectory to set.
        %   options: Any additional keyword arguments or choice.
        % 
        setParameters(self, trajectoryParams, options)
        
        % Computes the actual state to track for the given time.
        %
        % Should throw RTD:InvalidTrajectory if the trajectory isn't set.
        %
        % Arguments:
        %   time: Time to use to calculate the desired state for this trajectory
        %
        % Returns:
        %   rtd.entity.states.BaseEntityStateInstance: Desired state at the given time
        %
        command = getCommand(self, time)
    end
end