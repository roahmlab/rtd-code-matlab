classdef BaseDynamicsComponent < handle
    % BaseDynamicsComponent
    properties (Abstract)
        robot_info rtd.entity.components.BaseInfoComponent
        robot_state rtd.entity.components.BaseStateComponent
        controller rtd.entity.components.BaseControllerComponent
    end
    methods (Abstract)
        reset(self)
        move(self, t_move)
    end
end