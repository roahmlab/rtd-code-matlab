classdef ArmourController < rtd.entity.components.BaseControllerComponent & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle
    
    % Leftover Old Dependencies
    % robot_arm_LLC and uarmtd_robust_CBF_LLC
    % arminfo params by extension
    % arminfo M_min_eigenvalue???
    
    % Inherited properties that must be defined
    properties
        robot_info = armour.agent.ArmourAgentInfo.empty()
        robot_state = armour.agent.ArmourAgentState.empty()
        
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
        function self = ArmourController(arm_info, arm_state_component, optionsStruct, options)
            arguments
                arm_info armour.agent.ArmourAgentInfo
                arm_state_component armour.agent.ArmourAgentState
                optionsStruct.options struct = struct()
                options.use_true_params_for_robust
                options.use_disturbance_norm
                options.Kr
                options.alpha_constant
                options.V_max
                options.r_norm_threshold
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct.options, options);
            
            % Set base variables
            self.robot_info = arm_info;
            self.robot_state = arm_state_component;
            
            % Initialize
            % self.reset();
        end
        
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct.options struct = struct()
                options.use_true_params_for_robust
                options.use_disturbance_norm
                options.Kr
                options.alpha_constant
                options.V_max
                options.r_norm_threshold
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct.options, options);
            
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
            initial_traj = armour.trajectory.ZeroHoldArmTrajectory(self.robot_state.get_state);
            self.trajectories = {initial_traj};
        end
            
        
        function setTrajectory(self, trajectory)
            arguments
                self
                trajectory rtd.trajectory.Trajectory
            end
            % Add the trajectory if it is valid
            if trajectory.validate()
                self.trajectories = [self.trajectories, {trajectory}];
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
            startTime = self.robot_state.get_state().time;
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
            u_pos_ref = [reference_trajectory.position];
            u_vel_ref = [reference_trajectory.velocity];
            
            % check bound satisfaction
            self.vdisp('Running ultimate bound check!', 'INFO');
            
            % Absolute difference
            u_pos_diff = abs(u_pos_ref - [u_check.position]);
            u_vel_diff = abs(u_vel_ref - [u_check.velocity]);
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
    end
end
