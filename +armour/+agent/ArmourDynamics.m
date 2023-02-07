classdef ArmourDynamics < rtd.entity.components.BaseDynamicsComponent & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle
    
    % Torque dynamics with optional measurement noise
    % Leftover Old Dependencies
    % rnea_mass
    % rnea_coriolis
    % rnea_gravity
    % arminfo params by extension
    % and Controller by extension
    
    % Inherited properties that must be defined
    properties
        % General information of the robot arm
        robot_info = armour.agent.ArmourAgentInfo.empty()
        
        % The state of the arm
        robot_state = armour.agent.ArmourAgentState.empty()
        
        % The controller used
        controller = armour.agent.ArmourController.empty()
    end
    
    % Extra properties we define
    properties
        
        % Other properties
        time_discretization double = 0.01
        
        % Logging
        controller_log rtd.util.containers.VarLogger = rtd.util.containers.VarLogger.empty()

        % Measurement Noise (This probably should eventually be its own
        % class and type
        measurement_noise = struct
    end
    
    methods (Static)
        function options = defaultoptions()
            options.time_discretization = 0.01;
            options.measurement_noise_points = 0;
            options.measurement_noise_pos_sigma = 1e-4;
            options.measurement_noise_vel_sigma = 1e-4;
            options.log_controller = false;
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end
    
    methods
        function self = ArmourDynamics(arm_info, arm_state_component, controller_component, optionsStruct, options)
            arguments
                arm_info armour.agent.ArmourAgentInfo
                arm_state_component armour.agent.ArmourAgentState
                controller_component armour.agent.ArmourController
                optionsStruct struct = struct()
                options.time_discretization
                options.measurement_noise_points
                options.measurement_noise_pos_sigma
                options.measurement_noise_vel_sigma
                options.log_controller
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct, options);
            
            % set base variables
            self.robot_info = arm_info;
            self.robot_state = arm_state_component;
            self.controller = controller_component;
            
            self.reset()
        end
        
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.time_discretization
                options.measurement_noise_points
                options.measurement_noise_pos_sigma
                options.measurement_noise_vel_sigma
                options.log_controller
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct, options);
            
            % if we're going to log, set it up
            if options.log_controller
                self.controller_log = rtd.util.containers.VarLogger('input_time', ...
                                     'input', ...
                                     'nominal_input', ...
                                     'robust_input', ...
                                     'disturbance', ...
                                     'lyapunov', ...
                                     'r');
            end
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            % For integration output
            self.time_discretization = options.time_discretization;
            
            % Measurement noise generation parameters
            self.measurement_noise.points = options.measurement_noise_points;
            self.measurement_noise.pos_sigma = options.measurement_noise_pos_sigma;
            self.measurement_noise.vel_sigma = options.measurement_noise_vel_sigma;
        end
            
        
        function move(self, t_move)
            self.vdisp('Moving!', 'INFO')
            
            % get the current state
            state = self.robot_state.get_state;
            zcur = state.state;
            
            % call the ode solver to simulate agent
            [tout,zout] = self.integrator(@(t,z) self.dynamics(t,z),...
                                       [0 t_move], zcur) ;

            % save the motion data
            self.robot_state.commit_state_data(tout, zout);
            
            % If we have an active log, initialize trajectories to log
            if ~isempty(self.controller_log)
                uout = zeros(self.controller.n_inputs, size(tout, 2));
                nominal_out = zeros(size(uout));
                robust_out = zeros(size(uout));
                disturbance_out = zeros(size(uout));
                lyap_out = zeros(size(tout));
                r_out = zeros(self.robot_info.num_q, size(tout, 2));

                % store approximate inputs at each time:
                for j = 1:length(tout)
                    t = tout(j);
                    z_meas = zout(:,j);
                    [uout(:, j), nominal_out(:, j), robust_out(:, j),...
                     disturbance_out(:, j), lyap_out(:, j), r_out(:, j)] = ...
                     self.controller.getControlInputs(t, z_meas);
                end
                
                % Save to the log
                self.controller_log.add('input_time', state.time + tout, ...
                             'input', uout, ...
                             'nominal_input', nominal_out, ...
                             'robust_input', robust_out, ...
                             'disturbance', disturbance_out, ...
                             'lyapunov', lyap_out, ...
                             'r', r_out);
            end
        end
        
        function [t_out,z_out] = integrator(self, arm_dyn, t_span, z0)
            % Setup a consistent form of process noise if wanted
            if self.measurement_noise.points > 0
                self.generate_measurement_noise(t_span)
            end
            
            % setup ODE options
            t_meas = t_span(1):self.time_discretization:t_span(end);
            options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
            
            % call ODE solver
            [t_out, z_out] = ode15s(arm_dyn, t_meas, z0, options);
            
            % process
            t_out = t_out';
            z_out = z_out';
        end
        
        function zd = dynamics(self, t, state)
            % Rename variable for ease
            q = state(self.robot_state.position_indices);
            qd = state(self.robot_state.velocity_indices);
            
            % True dynamics
            [M, C, g] = self.calculate_dynamics(q, qd, self.robot_info.params.true);
            
            % Add measurement noise if desired
            if self.measurement_noise.points > 0
                [noise_pos, noise_vel] = self.get_measurement_noise(t);
                q = q + noise_pos;
                qd = qd + noise_vel;
            end
            
            z_meas = zeros(size(state));
            z_meas(self.robot_state.position_indices) = q;
            z_meas(self.robot_state.velocity_indices) = qd;
            
            u = self.controller.getControlInputs(t, z_meas);
            
            % update acceleration 
            qdd = M\(u-C*qd-g);

            % return dynamics
            zd = zeros(size(state));
            zd(self.robot_state.position_indices) = qd(:);
            zd(self.robot_state.velocity_indices) = qdd(:);
        end

        % Compute dynamic parameters given robot parameters
        function [M, C, g] = calculate_dynamics(~, q, qd, params)
            M = armour.legacy.dynamics.rnea_mass(q, params);
            C = armour.legacy.dynamics.rnea_coriolis(q, qd, params);
            g = armour.legacy.dynamics.rnea_gravity(q, params);
        end
        
        % Get the noise
        function [noise_pos, noise_vel] =  get_measurement_noise(self, t_vec)
            if self.measurement_noise.points == 0
                noise_pos = 0;
                noise_vel = 0;
            elseif self.measurement_noise.points == 1
                noise_pos = repmat(self.measurement_noise.pos, 1, length(t_vec));
                noise_vel = repmat(self.measurement_noise.vel, 1, length(t_vec));
            else
                noise_pos = interp1(self.measurement_noise.time, ...
                                self.measurement_noise.pos.', ...
                                t_vec).';
                noise_vel = interp1(self.measurement_noise.time, ...
                                self.measurement_noise.vel.', ...
                                t_vec).';
            end
        end

        % add measurement noise
        % make into a new class for interpolated process noise
        function generate_measurement_noise(self, t_span)
            % Time for interpolation
            self.measurement_noise.time ...
                = linspace(t_span(1), t_span(end), self.measurement_noise.points);
            
            % noise profile should try to match actual joint encoders
            % Position
            self.measurement_noise.pos              ...
                = self.measurement_noise.pos_sigma  ...
                * randn(self.robot_info.num_q, self.measurement_noise.points);
            
            % Velocity
            self.measurement_noise.vel              ...
                = self.measurement_noise.vel_sigma  ...
                * randn(self.robot_info.num_q, self.measurement_noise.points);
        end
        
        % Check that the controller wasn't giving bad torques
        function out = controller_input_check(self, t_check_step)
            % if not logging, skip
            if isempty(self.controller_log)
                self.vdisp('Not logging controller inputs, skipping input torque check!', 'INFO');
                out = false;
                return
            end
            
            % retrieve the last log entry
            entries = self.controller_log.get('input_time', 'input', flatten=false);
            t_input = entries.input_time{end};
            input = entries.input{end};
            
            % interpolate for the t_check_step and get agent input
            % trajectory interpolated to time
            t_check = t_input(1):t_check_step:t_input(end);
            u_check = interp1(t_input, input.', t_check).';

            % check torque bounds
            self.vdisp('Running input torque check!', 'INFO');
            u_exceeded = false(size(u_check));
            for idx=1:self.robot_info.num_q
                % Lower bound
                lb = u_check(idx,:) < self.robot_info.joints(idx).torque_limits(1);
                ub = u_check(idx,:) > self.robot_info.joints(idx).torque_limits(2);
                u_exceeded(idx,:) = lb + ub;
            end

            % Get out results
            out = any(u_exceeded, "all");
            if out
                [joint_idx_list, t_idx_list] = find(u_exceeded);
                for idx = 1:length(joint_idx_list)
                    t_idx = t_idx_list(idx);
                    joint_idx = joint_idx_list(idx);
                    % Format error message
                    msg = sprintf('t=%.2f, %d-torque exceeded: %.2f vs [%.2f, %.2f]', ...
                        t_check(t_idx), joint_idx, u_check(joint_idx, t_idx), ...
                        self.robot_info.joints(joint_idx).torque_limits(1), ...
                        self.robot_info.joints(joint_idx).torque_limits(2));
                    self.vdisp(msg, 'ERROR');
                end
            else
                self.vdisp('No inputs exceeded', 'INFO');
            end
        end
    end
end