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
    
    methods
        function obj = RobotInfo(inputArg1,inputArg2)
            %ROBOTINFO Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

