classdef BaseControllerComponent < handle
    % BaseControllerComponent Interface for an Entity Controller Component
    properties (Abstract)
        robot_info rtd.entity.components.BaseInfoComponent
        robot_state rtd.entity.components.BaseStateComponent
        trajectories(1,1) rtd.trajectory.TrajectoryContainer
        n_inputs uint32
    end
    methods (Abstract)
        reset(self)
        setTrajectory(self, trajectory)
        getControlInputs(self, varargin)
    end
end