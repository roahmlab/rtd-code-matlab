classdef ArmourDynamics < handle
    
    properties
        % General information of the robot arm
        arm_info RoboticsToolboxArmRobotInfo
        
        % The state of the arm
        arm_state
        
        % The controller used
        controller
        
        % Other properties
        time_discretization = 0.01
        
        % Logging
        % log_controller = false
        time = []
        inputs = []
        % clean
        % uout = []
        % nominal_out = []
        % robust_out = []
        % disturbance_out = []
        % lyp_out = []
        % r_out = []

        % Measurement Noise
        measurement_noise = {}
    end
    
    methods
        function self = ArmourDynamics(arm_info, arm_state_component, controller_component, options)
            arguments
                arm_info
                arm_state_component
                controller_component
                options.time_discretization = 0.01;
                options.measurement_noise_points = 0;
                options.measurement_noise_pos_sigma = 1e-4;
                options.measurement_noise_vel_sigma = 1e-4;
                % options.log_controller = false;
            end
            self.arm_info = arm_info;
            self.arm_state = arm_state_component;
            self.controller = controller_component;
            self.time_discretization = options.time_discretization;
            % self.log_controller = options.log_controller;
            
            self.measurement_noise.points = options.measurement_noise_points;
            self.measurement_noise.pos_sigma = options.measurement_noise_pos_sigma;
            self.measurement_noise.vel_sigma = options.measurement_noise_vel_sigma;
        end
        
        function move(self, t_move)
            % get the current state
            zcur = self.arm_state.state(:,end);
            
            % call the ode solver to simulate agent
            [tout,zout] = self.integrator(@(t,z) self.dynamics(t,z),...
                                       [0 t_move], zcur) ;

            % initialize trajectories to log
            uout = zeros(self.controller.n_inputs, size(tout, 2));
            nominal_out = zeros(size(uout));
            robust_out = zeros(size(uout));
            disturbance_out = zeros(size(uout));
            lyap_out = zeros(size(tout));
            r_out = zeros(self.arm_info.n_links_and_joints, size(tout, 2));

            % store approximate inputs at each time:
            for j = 1:length(tout)
                t = tout(j);
                z_meas = zout(:,j);
                [uout(:, j), nominal_out(:, j), robust_out(:, j),...
                 disturbance_out(:, j), lyap_out(:, j), r_out(:, j)] = ...
                 self.controller.getControlInputs(t, z_meas);
            end
            
            % Commit it
            self.arm_state.commit_state_data(tout, zout);
            self.commit_input_data(tout, uout);
        end
        
        % Convert to a mixin for loggin
        %function commit_input_data(self, T, uout, nominal_out, robust_out, disturbance_out, lyp_out, r_out)
        function commit_input_data(self, T, inputs)
            self.time = [self.time, self.time(end) + T(2:end)];
            self.inputs = [self.inputs, inputs(:,2:end)];
            % Add later
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
            q = state(self.arm_state.position_indices);
            qd = state(self.arm_state.velocity_indices);
            
            % True dynamics
            [M, C, g] = self.calculate_dynamics(q, qd, self.arm_info.params.true);
            
            % Add measurement noise if desired
            if self.measurement_noise.points > 0
                [noise_pos, noise_vel] ...
                    = match_trajectories(t ...
                    , self.measurement_noise.time, self.measurement_noise.pos ...
                    , self.measurement_noise.time, self.measurement_noise.vel, 'linear'); % linearly interpolate noise
                q = q + noise_pos;
                qd = qd + noise_vel;
            end
            
            z_meas = zeros(size(z));
            z_meas(self.arm_state.position_indices) = q;
            z_meas(self.arm_state.velocity_indices) = qd;
            
            u = self.controller.getControlInputs(t, z_meas);
            
            % update acceleration 
            qdd = M\(u-C*qd-g);

            % return dynamics
            zd = zeros(size(z));
            zd(self.arm_state.position_indices) = qd(:);
            zd(self.arm_state.velocity_indices) = qdd(:);
        end

        % Compute dynamic parameters given robot parameters
        function [M, C, g] = calculate_dynamics(~, q, qd, params)
            M = rnea_mass(q, params);
            C = rnea_coriolis(q, qd, params);
            g = rnea_gravity(q, params);
        end
        
        % add measurement noise
        % make into a mixin?
        function generate_measurement_noise(self, t_span)
            self.measurement_noise.time ...
                = linspace(t_span(1), t_span(end), self.measurement_noise.points);
            
            % noise profile should try to match actual joint encoders
            self.measurement_noise.pos              ...
                = self.measurement_noise.pos_sigma  ...
                * randn(self.arm_info.n_links_and_joints, self.measurement_noise.points);
            
            self.measurement_noise.vel              ...
                = self.measurement_noise.vel_sigma  ...
                * randn(self.arm_info.n_links_and_joints, self.measurement_noise.points);
        end
    end
end