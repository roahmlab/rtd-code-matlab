classdef BaseDynamicsComponent < handle
    % BaseDynamicsComponent
    properties (Abstract)
        robot_info EntityInfo
        robot_state EntityStateComponent
        controller BaseControllerComponent
    end
    methods (Abstract)
        reset(self)
        move(self, t_move)
    end
end