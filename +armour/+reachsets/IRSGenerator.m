classdef IRSGenerator < rtd.planner.reachsets.ReachSetGenerator
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
        use_robust_input
        robot
    end
    methods
        function self = IRSGenerator(robot, jrsGenerator, options)
            arguments
                robot armour.ArmourAgent
                jrsGenerator armour.reachsets.JRSGenerator
                options.use_robust_input(1,1) logical = true
                options.verboseLevel(1,1) rtd.util.types.LogLevel = "DEBUG"
            end
            self.robot = robot;
            self.jrsGenerator = jrsGenerator;
            self.use_robust_input = options.use_robust_input;
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
            
            self.vdisp("Generating input reachable set!", "INFO")

            % set up zeros and overapproximation of r
            self.vdisp("Set up zeros for overapproximation", 'TRACE')
            for j = 1:jrsInstance.n_q
                zero_cell{j, 1} = armour.pz_roahm.polyZonotope_ROAHM(0); 
                r{j, 1} = armour.pz_roahm.polyZonotope_ROAHM(0, [], self.robot.controller.ultimate_bound);
            end
            
            self.vdisp("Start RNEA", 'TRACE')
            for i = 1:jrsInstance.n_t
                self.vdisp("RNEA for nominal", 'TRACE')
                tau_nom{i, 1} = armour.legacy.dynamics.poly_zonotope_rnea( ...
                        jrsInstance.R{i}, ...
                        jrsInstance.R_t{i}, ...
                        jrsInstance.dq{i}, ...
                        jrsInstance.dq_a{i}, ...
                        jrsInstance.ddq_a{i}, ...
                        true, ...
                        self.robot.info.params.pz_nominal);
                if self.use_robust_input
                    self.vdisp("RNEA interval for robust input", 'TRACE')
                    [tau_int{i, 1}, f_int{i, 1}, n_int{i, 1}] = ...
                        armour.legacy.dynamics.poly_zonotope_rnea( ...
                            jrsInstance.R{i}, ...
                            jrsInstance.R_t{i}, ...
                            jrsInstance.dq{i}, ...
                            jrsInstance.dq_a{i}, ...
                            jrsInstance.ddq_a{i}, ...
                            true, ...
                            self.robot.info.params.pz_interval);
                    
                    self.vdisp("calculate w from robust controller", 'TRACE')
                    for j = 1:jrsInstance.n_q
                        w{i, 1}{j, 1} = tau_int{i, 1}{j, 1} - tau_nom{i, 1}{j, 1};
                        w{i, 1}{j, 1} = reduce(w{i, 1}{j, 1}, 'girard', self.robot.info.params.pz_interval.zono_order);
                    end
                    
                    self.vdisp("calculate v_cell", 'TRACE')
                    V_cell = armour.legacy.dynamics.poly_zonotope_rnea( ...
                        jrsInstance.R{i}, ...
                        jrsInstance.R_t{i}, ...
                        zero_cell, ...
                        zero_cell, ...
                        r, ...
                        false, ...
                        self.robot.info.params.pz_interval);
                    V{i, 1} = 0;
                    for j = 1:jrsInstance.n_q
                        V{i, 1} = V{i, 1} + 0.5.*r{j, 1}.*V_cell{j, 1};
                        V{i, 1} = reduce(V{i, 1}, 'girard', self.robot.info.params.pz_interval.zono_order);
                    end
                    V_diff{i, 1} = V{i, 1} - V{i, 1};
                    V_diff{i, 1} = reduce(V_diff{i, 1}, 'girard', self.robot.info.params.pz_interval.zono_order);
                    V_diff_int{i, 1} = interval(V_diff{i, 1});
                end
            end
            % % get max effect of disturbance r'*w... couple of ways to do this
            % % can overapproximate r and compute r'*w as a PZ
            % % can slice w first, then get the norm?
            % for i = 1:jrs_info.n_t
            %     r_dot_w{i, 1} = 0;
            %     for j = 1:jrs_info.n_q
            %         r_dot_w{i, 1} = r_dot_w{i, 1} + r{j, 1}.*w{i, 1}{j, 1};
            %         r_dot_w{i, 1} = reduce(r_dot_w{i, 1}, 'girard', P.params.pz_interval.zono_order);
            %     end
            % end
            % can get ||w|| <= ||\rho(\Phi)||, and compute the norm using interval arithmetic
            if self.use_robust_input
                self.vdisp("robust input", 'TRACE')
                for i = 1:jrsInstance.n_t
                    for j = 1:jrsInstance.n_q
                        w_int{i, 1}(j, 1) = interval(w{i, 1}{j, 1});
                    end
                    rho_max{i, 1} = norm(max(abs(w_int{i, 1}.inf), abs(w_int{i, 1}.sup)));
                end

                self.vdisp("robust input bound tortatotope", 'TRACE')
                % compute robust input bound tortatotope:
                for i = 1:jrsInstance.n_t
                    v_norm{i, 1} = (self.robot.controller.alpha_constant*V_diff_int{i, 1}.sup).*(1/self.robot.controller.ultimate_bound) + rho_max{i, 1};
%                     v_norm{i, 1} = reduce(v_norm{i, 1}, 'girard', P.agent_info.params.pz_interval.zono_order);
                end
            else
                for i = 1:jrsInstance.n_t
                    v_norm{i, 1} = 0;
                end
            end

            % compute total input tortatotope
            self.vdisp("Total input tortatotope", 'TRACE')
            for i = 1:jrsInstance.n_t
                for j = 1:jrsInstance.n_q
                    u_ub_tmp = tau_nom{i, 1}{j, 1} + v_norm{i, 1};
                    u_lb_tmp = tau_nom{i, 1}{j, 1} - v_norm{i, 1};
                    u_ub_tmp = remove_dependence(u_ub_tmp, jrsInstance.k_id(end));
                    u_lb_tmp = remove_dependence(u_lb_tmp, jrsInstance.k_id(end));
                    u_ub_buff = sum(abs(u_ub_tmp.Grest));
                    u_lb_buff = -sum(abs(u_lb_tmp.Grest));
                    u_ub{i, 1}{j, 1} = armour.pz_roahm.polyZonotope_ROAHM(u_ub_tmp.c + u_ub_buff, u_ub_tmp.G, [], u_ub_tmp.expMat, u_ub_tmp.id) - self.robot.info.joints(j).torque_limits(2);
                    u_lb{i, 1}{j, 1} = -1*armour.pz_roahm.polyZonotope_ROAHM(u_lb_tmp.c + u_lb_buff, u_lb_tmp.G, [], u_lb_tmp.expMat, u_lb_tmp.id) + self.robot.info.joints(j).torque_limits(1);
                end
            end
            
            % Save the generated reachable sets into the IRSInstance
            reachableSet = armour.reachsets.IRSInstance(u_ub, u_lb, jrsInstance, self.get_vdisplevel);
        end
    end
end