classdef JLSGenerator < rtd.planner.reachsets.ReachSetGenerator
    % InputReachableSet
    % This generates the upper and lower bound reachable sets on the input,
    % and creates an IRSInstance object.

    % Required Abstract Properties
    properties
        cache_max_size = 1
    end

    % Additional Properties
    properties
        jrsGenerator
        robot
    end
    methods
        function self = JLSGenerator(robot, jrsGenerator, options)
            arguments
                robot armour.ArmourAgent
                jrsGenerator armour.reachsets.JRSGenerator
                options.verboseLevel(1,1) rtd.util.types.LogLevel = "DEBUG"
            end
            self.robot = robot;
            self.jrsGenerator = jrsGenerator;
            self.set_vdisplevel(options.verboseLevel);
        end
    end
    methods (Access=protected)
        
        % Obtains the relevant reachable set for the robotstate provided
        % and outputs the singular instance of a reachable set.
        % Returns IRSInstance
        function reachableSet = generateReachableSet(self, robotState)
            % Computes the forward kinematics and occupancy
            
            % First get the JRS (allow the use of a cached value if it
            % exists)
            jrsInstance = self.jrsGenerator.getReachableSet(robotState, ignore_cache=false);

            joint_state_limits = [self.robot.info.joints.position_limits];
            joint_speed_limits = [self.robot.info.joints.velocity_limits];
            joint_limit_infs = isinf(joint_state_limits);
            speed_limit_infs = isinf(joint_speed_limits);
            joint_state_limits(1,joint_limit_infs(1,:)) = -200*pi;
            joint_state_limits(2,joint_limit_infs(2,:)) = +200*pi;            
            joint_speed_limits(1,speed_limit_infs(1,:)) = -200*pi;
            joint_speed_limits(2,speed_limit_infs(2,:)) = +200*pi;
            
            self.vdisp("Generating joint limit set!", "INFO")

            % joint limit constraint setup
            for i = 1:jrsInstance.n_t
                for j = 1:jrsInstance.n_q
                    q_lim_tmp = jrsInstance.q{i, 1}{j, 1};
                    dq_lim_tmp = jrsInstance.dq{i, 1}{j, 1};
                    q_lim_tmp = remove_dependence(q_lim_tmp, jrsInstance.k_id(end));
                    dq_lim_tmp = remove_dependence(dq_lim_tmp, jrsInstance.k_id(end));
                    q_buf = sum(abs(q_lim_tmp.Grest));
                    dq_buf = sum(abs(dq_lim_tmp.Grest));
                    q_ub{i, 1}{j, 1} = armour.pz_roahm.polyZonotope_ROAHM(q_lim_tmp.c + q_buf, q_lim_tmp.G, [], q_lim_tmp.expMat, q_lim_tmp.id) - joint_state_limits(2, j);
                    q_lb{i, 1}{j, 1} = -1*armour.pz_roahm.polyZonotope_ROAHM(q_lim_tmp.c + q_buf, q_lim_tmp.G, [], q_lim_tmp.expMat, q_lim_tmp.id) + joint_state_limits(1, j);
                    dq_ub{i, 1}{j, 1} = armour.pz_roahm.polyZonotope_ROAHM(dq_lim_tmp.c + dq_buf, dq_lim_tmp.G, [], dq_lim_tmp.expMat, dq_lim_tmp.id) - joint_speed_limits(2, j);
                    dq_lb{i, 1}{j, 1} = -1*armour.pz_roahm.polyZonotope_ROAHM(dq_lim_tmp.c + dq_buf, dq_lim_tmp.G, [], dq_lim_tmp.expMat, dq_lim_tmp.id) + joint_speed_limits(1, j);
                end
            end
            
            % Save the generated reachable sets into the IRSInstance
            reachableSet = armour.reachsets.JLSInstance(q_ub, q_lb, dq_ub, dq_lb, jrsInstance, self.get_vdisplevel);
        end
    end
end