classdef BaseControllerComponent < handle
    % BaseControllerComponent Interface for an Entity Controller Component
    properties (Abstract)
        robot_info EntityInfo
        robot_state EntityState
        trajectories cell
    end
    methods (Abstract)
        reset(self)
        set_trajectory(self, trajectory)
        get_control_inputs(self, varargin)
    end
end