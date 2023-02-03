classdef TrajectoryFactory < handle
% Base class for the Trajectory factory object
%
% This should be used to initialize desired instaces of
% :mat:class:`+rtd.+planner.+trajectory.Trajectory`.
%
% Note:
%   `createTrajectory(self, robotState, rsInstances, trajectoryParams,
%   options)` should be implemented. Check help for more information
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-01-25
% Last Revised: 2023-02-02
% 
% See also createTrajectory, rtd.planner.trajectory.Trajectory,
% rtd.planner.trajopt.TrajOptProps
%
% --- More Info ---
% 

    properties
        % Properties from the trajectory optimization, which also describe
        % properties for a trajectory.
        trajOptProps rtd.planner.trajopt.TrajOptProps %
    end

    methods (Abstract)
        % Factory method to create the trajectory.
        %
        % This method constructs any relevant Trajectory objects and fully
        % parameterizes them if desired. Additional options can be set in
        % here, or handled in the class properties.
        %
        % Arguments:
        %   robotState (rtd.entity.states.EntityState): Initial state of the robot
        %   rsInstances (struct): Optional struct holding instances of reachablesets for the given state
        %   trajectoryParms: Optional parameters to fully parameterize the trajectories generated
        %   options: Additional optional keyword arguments if desired
        %
        % Returns:
        %   rtd.planner.trajectory.Trajectory: Desired Trajectory Object
        %
        trajectory = createTrajectory(self, robotState, rsInstances, trajectoryParams, options)
    end
end