classdef ArmRobotState < RobotState
    % ArmRobotState
    % Information on the atomic state of the robot at a given point of
    % time. Each hard instance of this (unique object, not seperate
    % handles to the same underlying object) will have a unique uuid.
    properties
        q
        q_dot
        q_ddot
    end
end