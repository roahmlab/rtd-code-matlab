classdef ArmRobotState < RobotState
    properties
        q
        q_dot
        q_ddot
    end
    methods
        function self = ArmRobotState()
            self.q = [0];
            isstruct(self)
            isempty(self.q_dot)
        end
    end
end