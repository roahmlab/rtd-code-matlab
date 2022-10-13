classdef ArmRobotInfo < RobotInfo
    % ArmRobotInfo
    % Unchanging parts of the ArmRobot that should only be set in the
    % beginning. If you find any parts of this changing, it should either
    % be a part of the RobotModel or RobotState!
    
    properties
        % kinematic and inertial params
        params % {nominal, true, interval, pz_interval, pz_nominal}
        link_poly_zonotopes
        LLC_info
        joint_input_limits
    end
end

