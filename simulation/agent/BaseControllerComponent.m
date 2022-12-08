classdef BaseControllerComponent < handle
    properties
        robot_info
        robot_state
    end
    methods
        get_control_inputs(self, varargin)
    end
end