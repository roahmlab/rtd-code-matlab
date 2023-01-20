classdef JointReachableSetsOnline < ReachableSets & rtd.core.mixins.NamedClass
    % JointReachableSetsOnline
    % This does the online computation of joint reachable sets. It then
    % generates a JRSInstance object.
    properties
        cache_max_size = 1
        taylor_degree
        add_ultimate_bound
        traj_type
    end
    methods
        function self = JointReachableSetsOnline( ...
                    robot, options ...
                )
                arguments
                    robot ArmourAgent
                    options.taylor_degree = 1
                    options.add_ultimate_bound = true
                    options.traj_type {mustBeMember(options.traj_type,{'orig','bernstein'})} = 'orig'
                end
            
            self.robot = robot;
            self.taylor_degree = options.taylor_degree;
            self.add_ultimate_bound = options.add_ultimate_bound;
            self.traj_type = options.traj_type;
        end
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Wraps create_jrs_online
        % Returns JRSInstance
        function reachableSet = generateReachableSet(self, robotState)
            % The way I choose to wrap this is not the way it has to be
            % done. I choose to have it create the object first, then
            % initialize the remaining properties from this loaded data.
            rs = JRSInstance;
            
            self.vdisp("Following message is from create_jrs_online");
            % Generate it online as per the original implementation
            [rs.q_des, rs.dq_des, rs.ddq_des,            ...
                rs.q, rs.dq, rs.dq_a, rs.ddq_a,             ...
                rs.R_des, rs.R_t_des, rs.R, rs.R_t,         ...
                rs.jrs_info] = create_jrs_online(  ...
                    robotState.q,               ...
                    robotState.q_dot,           ...
                    robotState.q_ddot,          ...
                    [self.robot.info.joints.axes], ...
                    self.taylor_degree,         ...
                    self.traj_type,                     ...
                    self.add_ultimate_bound);
            
            % Initialize this particular instance and return
            rs.initialize(self.traj_type);
            reachableSet = rs;
        end
    end
end