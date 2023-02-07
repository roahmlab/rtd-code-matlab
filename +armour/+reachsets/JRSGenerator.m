classdef JRSGenerator < rtd.planner.reachsets.ReachSetGenerator
    % JointReachableSetsOnline
    % This does the online computation of joint reachable sets. It then
    % generates a JRSInstance object.

    % Required Abstract Properties
    properties
        cache_max_size = 1
    end

    % Additional Properties
    properties
        taylor_degree
        add_ultimate_bound
        traj_type
        joint_axes
        controller
    end

    % Properties for the generator
    methods
        function self = JRSGenerator(robot, options)
            arguments
                robot armour.ArmourAgent
                options.taylor_degree(1,1) double {mustBeInteger} = 1
                options.add_ultimate_bound(1,1) logical = true
                options.traj_type {mustBeMember(options.traj_type,{'piecewise','bernstein'})} = 'piecewise'
                options.verboseLevel(1,1) rtd.util.types.LogLevel = "DEBUG"
            end
            
            self.joint_axes = [robot.info.joints.axes];
            self.controller = robot.controller;
            self.taylor_degree = options.taylor_degree;
            self.add_ultimate_bound = options.add_ultimate_bound;
            self.traj_type = options.traj_type;
            self.set_vdisplevel(options.verboseLevel);
        end
    end

    % Generate a new JRS
    methods (Access=protected)
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Wraps create_jrs_online
        % Returns JRSInstance
        function reachableSet = generateReachableSet(self, robotState)
            % The way I choose to wrap this is not the way it has to be
            % done. I choose to have it create the object first, then
            % initialize the remaining properties from this loaded data.
            rs = armour.reachsets.JRSInstance;

            % compat
            traj_type_adapt = self.traj_type;
            if strcmp(traj_type_adapt, 'piecewise')
                traj_type_adapt = 'orig';
            end
            
            self.vdisp("Generating joint reachable set!", "INFO")

            self.vdisp("The following message is from create_jrs_online", 'INFO');
            % Generate it online as per the original implementation
            [rs.q_des, rs.dq_des, rs.ddq_des,            ...
                rs.q, rs.dq, rs.dq_a, rs.ddq_a,             ...
                rs.R_des, rs.R_t_des, rs.R, rs.R_t,         ...
                rs.jrs_info] = armour.legacy.create_jrs_online(  ...
                    robotState.q,               ...
                    robotState.q_dot,           ...
                    robotState.q_ddot,          ...
                    self.joint_axes, ...
                    self.taylor_degree,         ...
                    traj_type_adapt,                     ...
                    self.add_ultimate_bound, ...
                    self.controller);
            
            % Initialize this particular instance and return
            rs.initialize(self.traj_type);
            reachableSet = rs;
        end
    end
end