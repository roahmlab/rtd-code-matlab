classdef BaseDynamicsComponent < handle
    % BaseDynamicsComponent
    properties (Abstract)
        robot_info rtd.core.components.BaseInfoComponent
        robot_state rtd.core.components.BaseStateComponent
        controller rtd.core.components.BaseControllerComponent
    end
    methods (Abstract)
        reset(self)
        move(self, t_move)
    end
end