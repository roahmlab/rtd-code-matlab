classdef BaseControllerComponent < handle
    % BaseControllerComponent Interface for an Entity Controller Component
    properties (Abstract)
        robot_info EntityInfo
        robot_state EntityStateComponent
        trajectories cell
        n_inputs uint32
    end
    methods (Abstract)
        reset(self)
        setTrajectory(self, trajectory)
        getControlInputs(self, varargin)
    end
end