classdef RefineController < rtd.entity.components.BaseControllerComponent & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle

    properties
        robot_info = refine.agent.RefineAgentInfo.empty()
        robot_state = refine.agent.RefineAgentState.empty()
        
        n_inputs uint32 = 0
    end
    
    % Extra properties we define
    properties
        ultimate_bound double
        ultimate_bound_position double
        ultimate_bound_velocity double

        Kr double
        alpha_constant double
        V_max double
        r_norm_threshold double
        
        trajectories = {}
    end
    
    methods (Static)
        function options = defaultoptions()
            % Configurable options for this component
            options.use_true_params_for_robust = false;
            options.use_disturbance_norm = true;
            options.Kr = 10;
            options.alpha_constant = 1;
            options.V_max = 3.1e-7;
            options.r_norm_threshold = 0;
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end

    methods

        function self = ArmourController(robot_info, robot_state_component, optionsStruct, options)
            arguments
                robot_info armour.agent.ArmourAgentInfo
                robot_state_component armour.agent.ArmourAgentState
                optionsStruct struct = struct()
                options.use_true_params_for_robust
                options.use_disturbance_norm
                options.Kr
                options.alpha_constant
                options.V_max
                options.r_norm_threshold
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct, options);
            
            % Set base variables
            self.robot_info = robot_info;
            self.robot_state = robot_state_component;
            
            % Initialize
            self.reset();
        end

        function [delta, w_cmd, r_err, h_err, u_err, desired_front_lat_force, desired_lon_force] = FL_LLC(A,t,z,T,U,Z)
            h = z(3);
            u = z(4);
            v = z(5);
            r = z(6);
%             w = z(7); % tire speed is no longer used
            r_err_sum = z(8);
            h_err_sum = z(9);
            u_err_sum = z(10);
            
            
            % desired trajectory
            ud =interp1(T,U(1,:),t,'linear'); 
            uddot =interp1(T,U(4,:),t,'linear');
            rd = interp1(T,U(3,:),t,'linear');
            rddot =interp1(T,U(6,:),t,'linear');
            hd = interp1(T,Z(3,:),t,'linear');
            r_dot_des = A.Kr*(rd -r) + rddot + A.Kh*(hd -h);
            u_dot_des = A.Ku*(ud -u) + uddot;
            
            % lateral force computation for steering angle delta
            r_err = r - rd;
            h_err = h - hd;
            err_term = A.Kr* r_err + A.Kh*h_err;
            kappa = A.kappaP + A.kappaI* (r_err_sum + h_err_sum);
            phi = A.phiP + A.phiI*(r_err_sum + h_err_sum);
            tau_r = -(kappa*A.Mr + phi) *err_term;
            vf = v + A.lf*r;
            vr = v - A.lr*r;
            alphar = - vr / max(u, A.u_cri*1.1); % modify denominator for numerical stability
            Fywr = A.Car1*alphar;
            desired_front_lat_force = (A.Izz*(r_dot_des + tau_r) + A.lr*Fywr)/A.lf;
            delta = desired_front_lat_force/A.Caf1 + vf/u;

            

            % longitudina force computation for wheel speed w_cmd      
            kappaU = A.kappaPU + A.kappaIU*(u_err_sum);
            phiU = A.phiPU + A.phiIU*(u_err_sum);
            u_err = u - ud;
            err_term = A.Ku*u_err;
            tau_u = -(kappaU*A.Mu + phiU) * err_term;
            desired_lon_force = (u_dot_des + tau_u - v*r)*A.m;
            Cbf = A.m * A.grav_const * A.lr / A.l * A.mu_bar;
            if uddot <= 0 
                w_cmd = (desired_lon_force/Cbf * u + u) / A.rw;
            else
                w_cmd = u/A.rw/(1-desired_lon_force/Cbf);
            end
        end
        function [delta, w_cmd, vlo, rlo, u_err] = Low_Spd_LLC(A,t,z,T,U,Z)
            u = z(4);
            u_err_sum = z(10);
            mr = A.lf/A.l *A.m;
            

            ud =interp1(T,U(1,:),t,'linear'); 
            uddot =interp1(T,U(4,:),t,'linear');
            rd = interp1(T,U(3,:),t,'linear');
            delta = rd *(A.l+A.Cus*u^2/A.grav_const) / max(u,0.01); % modify denominator for numerical stability
            rlo = delta*u/(A.l+A.Cus*u^2/A.grav_const);
            vlo = rlo*(A.lr - u^2*mr/A.Car1);
            
            u_dot_des = A.Ku*(ud -u) + uddot;   
            kappaU = A.kappaPU + A.kappaIU*(u_err_sum);
            phiU = A.phiPU + A.phiIU*(u_err_sum);
            u_err = u - ud;
            err_term = A.Ku*u_err;
            Mu_lo = A.max_Fx_uncertainty_braking / A.m;
            tau_u = -(kappaU*Mu_lo + phiU) * err_term;
            desired_lon_force = (u_dot_des + tau_u - vlo*rlo)*A.m;
            Cbf = A.m * A.grav_const * A.lr / A.l * A.mu_bar;
            if uddot <= 0 
                w_cmd = (desired_lon_force/Cbf *  u+ u) / A.rw;
            else
                w_cmd = u/A.rw/(1-desired_lon_force/Cbf);
            end
            
        end

        function [u, info] = getControlInputs(self, t, z_meas, options)
            arguments
                self armour.agent.ArmourController
                t
                z_meas
                options.doMoreLogging = false
            end

            % get actual robot state
            z = z_meas(:);
            position = z(self.robot_state.position_indices);
            velocity = z(self.robot_state.velocity_indices);

            % Prepare trajectory
            trajectory = self.trajectories{end};
            startTime = trajectory.startState.time;
            target = trajectory.getCommand(startTime + t);

            % error terms
            err = target.position - position;
            d_err = target.velocity - velocity;

            % modified reference trajectory
            vel_ref = target.velocity + self.Kr * err;
            acc_ref = target.acceleration + self.Kr * d_err;

            r = d_err + self.Kr * err;

            % Nominal controller
            tau = self.rnea( ...
                    position, velocity, ...
                    vel_ref, acc_ref, ...
                    true, self.robot_info.params.nominal);

            % robust input
            if norm(r) > self.r_norm_threshold
                if self.instanceOptions.use_true_params_for_robust
                    % calculate true disturbance
                    disturbance = self.rnea( ...
                            position, velocity, ...
                            vel_ref, acc_ref, ...
                            true, self.robot_info.params.true) - tau;

                    % bounds on max disturbance
                    if self.instanceOptions.use_disturbance_norm
                        norm_rho = norm(disturbance);
                    else
                        rho = abs(r)'*abs(disturbance);
                    end
                    
                    % compute true Lyapunov function
                    V_tmp = self.rnea( ...
                            position, zeros(self.robot_info.num_q, 1), ...
                            zeros(self.robot_info.num_q, 1), r, ...
                            false, self.robot_info.params.true);
                    V = 0.5*r'*V_tmp;
                else
                    % calculate interval disturbance
                    disturbance = self.rnea( ...
                            position, velocity, ...
                            vel_ref, acc_ref, ...
                            true, self.robot_info.params.interval) - tau;
                    
                    % bounds on max disturbance
                    if self.instanceOptions.use_disturbance_norm
                        Phi_min = abs(infimum(disturbance));
                        Phi_max = abs(supremum(disturbance));
                        norm_rho = norm(max(Phi_min, Phi_max));
                    else
                        rho = supremum(abs(r)'*abs(disturbance));
                    end
                    
                    % compute interval Lyapunov function
                    V_tmp = self.rnea( ...
                            position, zeros(self.robot_info.num_q, 1), ...
                            zeros(self.robot_info.num_q, 1), r, ...
                            false, self.robot_info.params.interval);
                    V_int = 0.5*r'*V_tmp;
                    V = supremum(V_int);
                end
                
                % assemble input
                h = -V + self.V_max; % value of CBF function

                if self.instanceOptions.use_disturbance_norm
                    lambda = max(0, (-self.alpha_constant * h)/norm(r)^2 + norm_rho/norm(r)); % coefficient on r
                else
                    lambda = max(0, (-self.alpha_constant * h + rho)/norm(r)^2);
                end

                v = lambda*r; % positive here! because u = tau + v in this code instead of tau - v
            else
                v = zeros(size(r));
                V = 0;
                if ~self.instanceOptions.use_true_params_for_robust
                    V_int = interval(0, 0);
                    disturbance = interval(0, 0);                    
                end
            end
    
            % combine nominal and robust inputs
            u = tau + v;

            % Info
            info.nominal_input = tau;
            info.robust_input = v;

            % output more for logging if desired
            if options.doMoreLogging
                if ~self.instanceOptions.use_true_params_for_robust
                    true_disturbance = self.rnea(position, velocity, vel_ref, acc_ref, true, self.robot_info.params.true) - tau;
                    true_V_tmp = self.rnea(position, zeros(self.robot_info.num_q, 1), zeros(self.robot_info.num_q, 1), r, false, self.robot_info.params.true);
                    true_V = 0.5*r'*true_V_tmp;
                    int_V = V_int;
                    int_disturbance = disturbance;
                else
                    if norm(r) <= self.r_norm_threshold
                        disturbance = self.rnea(position, velocity, vel_ref, acc_ref, true, self.robot_info.params.true) - tau;
                    end
                    true_disturbance = disturbance;
                    true_V = V;
                    int_V = V;
                    int_disturbance = disturbance;
                end
                info.disturbance = true_disturbance;
                info.lyapunov = true_V;
                info.r = r;
                info.int_disturbance = int_disturbance;
                info.int_lyapunov = int_V;
            end
        end
        
        % Check the controller against the controller log
        function out = ultimate_bound_check(self, t_check_step, controller_log)
            % if no log, skip
            if isempty(controller_log)
                self.vdisp('Not logging controller inputs, skipping ultimate bound check!', 'INFO');
                out = false;
                return
            end
            
            % retrieve the last log entry
            entries = controller_log.get('input_time', 'input', flatten=false);
            t_input = entries.input_time{end};
            input = entries.input{end};

            % interpolate for the t_check_step and get agent input
            % trajectory interpolated to time
            t_check = t_input(1):t_check_step:t_input(end);
            u_check = self.robot_state.get_state(t_check);
            % Get the reference trajectory
            trajectory = self.trajectories{end};
            reference_trajectory = arrayfun(@(t)trajectory.getCommand(t), t_check);
            u_pos_ref = [reference_trajectory.q];
            u_vel_ref = [reference_trajectory.q_dot];
            
            % check bound satisfaction
            self.vdisp('Running ultimate bound check!', 'INFO');
            
            % Absolute difference
            u_pos_diff = abs(u_pos_ref - u_check.q);
            u_vel_diff = abs(u_vel_ref - u_check.q_dot);
            position_exceeded = u_pos_diff > self.ultimate_bound_position;
            velocity_exceeded = u_vel_diff > self.ultimate_bound_velocity;
            
            % Get out results
            out = any(position_exceeded, 'all') || any(velocity_exceeded, 'all');
            if out
                % Position ultimate bound exceeded in these positions
                [joint_idx_list, t_idx_list] = find(position_exceeded);
                for idx = 1:length(joint_idx_list)
                    t_idx = t_idx_list(idx);
                    joint_idx = joint_idx_list(idx);
                    % Format error message
                    msg = sprintf('t=%.2f, %d-position bound exceeded: %.5f vs +-%.5f',...
                        t_check(t_idx), joint_idx, u_pos_diff(joint_idx, t_idx), ...
                        self.ultimate_bound_position);
                    self.vdisp(msg, 'ERROR');
                end
                
                % Velocity ultimate bound exceeded in these positions
                [joint_idx_list, t_idx_list] = find(velocity_exceeded);
                for idx = 1:length(joint_idx_list)
                    t_idx = t_idx_list(idx);
                    joint_idx = joint_idx_list(idx);
                    % Format error message
                    msg = sprintf('t=%.2f, %d-velocity bound exceeded: %.5f vs +-%.5f',...
                        t_check(t_idx), joint_idx, u_vel_diff(joint_idx, t_idx), ...
                        self.ultimate_bound_velocity);
                    self.vdisp(msg, 'ERROR');
                end
            else
                self.vdisp('No ultimate bound exceeded', 'INFO');
            end
        end

        % create a masked rnea call to handle transmission inertia
        function [u, f, n] = rnea(self, q, qd, q_aux_d, qdd, use_gravity, robot_params)
            [u, f, n] = armour.legacy.dynamics.rnea(q, qd, q_aux_d, qdd, use_gravity, robot_params);
            u = u(:) + qdd(:) .* self.robot_info.transmission_inertia(:);
        end

        function setTrajectory(self, trajectory)
            arguments
                self
                trajectory rtd.planner.trajectory.Trajectory
            end
            % Add the trajectory if it is valid
            if trajectory.validate()
                self.trajectories = [self.trajectories, {trajectory}];
            end
        end

        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.use_true_params_for_robust
                options.use_disturbance_norm
                options.Kr
                options.alpha_constant
                options.V_max
                options.r_norm_threshold
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct, options);
            
            % Set component dependent variables
            self.n_inputs = self.robot_info.num_q;
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);

            % flatten out pertinent properties
            self.Kr = options.Kr;
            self.alpha_constant = options.alpha_constant;
            self.V_max = options.V_max;
            self.r_norm_threshold = options.r_norm_threshold;
            
            % Compute ultimate bounds
            if isprop(self.robot_info, 'M_min_eigenvalue')
                self.ultimate_bound = sqrt(2*self.V_max/self.robot_info.M_min_eigenvalue);
                self.ultimate_bound_position = (1/self.Kr)*self.ultimate_bound;
                self.ultimate_bound_velocity = 2*self.ultimate_bound;
                self.vdisp(sprintf('Computed ultimate bound of %.3f', self.ultimate_bound), 'GENERAL');
            else
                warning('No minimum eigenvalue of agent mass matrix specified, can not compute ultimate bound');
            end
            
            % Create the initial trajectory
%             initial_traj = armour.trajectory.ZeroHoldArmTrajectory(self.robot_state.get_state);
%             self.trajectories = {initial_traj};
        end
    end
end
