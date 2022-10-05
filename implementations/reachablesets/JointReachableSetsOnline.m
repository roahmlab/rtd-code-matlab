classdef JointReachableSetsOnline < ReachableSets & NamedClass
    % JointReachableSetsOnline
    % This either encapsulates the reachable sets in memory, or enables the
    % online computation of reachable sets. It acts as a generator for a
    % single instance of ReachableSet
    properties
        cache_max_size = 1
        robotInfo
        taylor_degree
        add_ultimate_bound
    end
    methods
        % An example constructor, but can take anything needed
        function self = JointReachableSetsOnline( ...
                    robotInfo ...
                )
            self.robotInfo = robotInfo;
            % TODO
            self.taylor_degree = 1;
            self.add_ultimate_bound = true;
        end
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns ReachbleSet
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
                    self.robotInfo.params.pz_nominal.joint_axes, ...
                    self.taylor_degree,         ...
                    'orig',                     ...
                    self.add_ultimate_bound);
            
            % Initialize this particular instance and return
            rs.initialize();
            reachableSet = rs;
        end
    end
end