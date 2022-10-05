classdef ArmRobotInfo < RobotInfo
    %ROBOTINFO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % kinematic and inertial params
        params % {nominal, true, interval, pz_interval, pz_nominal}
        link_poly_zonotopes
        LLC_info
        joint_input_limits
    end
end

