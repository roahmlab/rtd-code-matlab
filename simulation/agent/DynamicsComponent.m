classdef DynamicsComponent < handle
    %DynamicsComponent
    % TODO Organize & Complete
    
    properties
        % General information of the robot arm
        arm_info RoboticsToolboxArmRobotInfo
        
        arm_state_component
        
        controller_component
    end
    
    methods
        function self = DynamicsComponent(arm_info,arm_state_component,controller_component)
            self.arm_info = arm_info;
            self.arm_state_component = arm_state_component;
            self.controller_component = controller_component;
        end
        
        %% move
        function move(A,t_move,T_ref,U_ref,Z_ref)
            % method: move(t_move,T_ref,U_ref,Z_ref)
            %
            % Moves the agent for the duration t_move using the nominal
            % inputs U_ref and nominal trajectory Z_ref that are indexed by
            % the nominal time T_ref.
            %
            % This method assumes that the input is zero-order hold, and
            % the input corresponding to the last time index is zero; this
            % is why the last old input is discarded when the input is
            % updated. Similarly, this method assumes that the nominal time
            % starts at 0.
            
            A.vdisp('Moving!',5)
            
            % set up default reference trajectory
            if nargin < 5
                Z_ref = [] ;
            end
            
            % get the time, input, and reference trajectory to use for
            % moving the agent
            [T_used,U_used,Z_used] = self.controller.move_setup(t_move,T_ref,U_ref,Z_ref) ;
            
            % get the current state
            zcur = A.state(:,end) ;
            
            % call the ode solver to simulate agent
            [tout,zout] = A.integrator(@(t,z) A.dynamics(t,z,T_ref,U_ref,Z_ref),...
                                       [0 t_move], zcur) ;
            
            A.commit_move_data(tout,zout,T_used,U_used,Z_used) ;
        end
        
        function move_random(A)
            A.move(1,[0 1],zeros(A.n_inputs,2), 2.*rand(A.n_states,2) - 1)
        end
        
        %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            % get desired torques and bound them
            u = A.LLC.get_control_inputs(A,t,z,T,U,Z) ;
            
            u = bound_array_elementwise(u,...
                A.joint_input_limits(1,:)',A.joint_input_limits(2,:)') ;
            
            % get joint speeds
            qd = z(A.joint_speed_indices) ;
            qd = bound_array_elementwise(qd,...
                A.joint_speed_limits(1,:)',A.joint_speed_limits(2,:)') ;
            
            % preallocate dynamics
            zd = zeros(A.n_states,1) ;
            
            if A.use_robotics_toolbox_model_for_dynamics_flag
                % get joint accelerations
                q = z(A.joint_state_indices) ;
                qdd = A.robotics_toolbox_model.forwardDynamics(q,qd) ;
            else
                % assume the inputs are the joint accelerations
                qdd = u(:) ;
            end
            
            % compute dynamics
            zd(A.joint_state_indices) = qd(:) ;
            zd(A.joint_speed_indices) = qdd(:) ;
        end
        
        %% integrator
        function [t_out,z_out] = integrator(A,arm_dyn,t_span,z0)
            % [tout,zout] = A.integrator(arm_dynamics,tspan,z0)
            %
            % RK4 integration with joint limits and speed limits enforced.
            
            % create time vector
            dt = A.integrator_time_discretization ;
            t_out = t_span(1):dt:t_span(end) ;
            if t_out(end) < t_span(end)
                t_out = [t_out, t_span(end)] ;
            end
            
            % preallocate trajectory output
            N_t = size(t_out,2) ;
            z_out = [z0(:), nan(A.n_states,N_t-1)] ;
            
            % run integration loop
            for tidx = 2:N_t
                % get previous state
                z_cur = z_out(:,tidx-1) ;
                t_cur = t_out(tidx-1) ;
                
                % compute RK4 terms
                k1 = arm_dyn(t_cur, z_cur) ;
                k2 = arm_dyn(t_cur + dt/2, z_cur + dt*k1/2) ;
                k3 = arm_dyn(t_cur + dt/2, z_cur + dt*k2/2) ;
                k4 = arm_dyn(t_cur + dt, z_cur + dt*k3) ;
                
                % compute summed term
                dzdt = (1/6)*(k1 + 2*k2 + 2*k3 + k4) ;
                
                % compute next state
                z_new = z_cur + dt*dzdt ;
                
                % apply state limits
                joint_values = z_new(A.joint_state_indices)' ;
                joint_values = max([joint_values ; A.joint_state_limits(1,:)],[],1) ;
                joint_values = min([joint_values ; A.joint_state_limits(2,:)],[],1) ;
                z_new(A.joint_state_indices) = joint_values ;
                
                % apply speed limits
                joint_speeds = z_new(A.joint_speed_indices)' ;
                joint_speeds = max([joint_speeds; A.joint_speed_limits(1,:)],[],1) ;
                joint_speeds = min([joint_speeds; A.joint_speed_limits(2,:)],[],1) ;
                z_new(A.joint_speed_indices) = joint_speeds ;
                
                % save new state
                z_out(:,tidx) = z_new(:) ;
            end
        end
        
        %% ARMOUR
        %% integrator
        function [t_out,z_out] = integrator(A,arm_dyn,t_span,z0)
            
            if A.add_measurement_noise_
                A.add_measurement_noise(t_span);
            end

            % setup ODE options
            dt = A.traj_check_time_discretization;
            t_span = t_span(1):dt:t_span(2);
            options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
            
            % call ODE solver
            [t_out, z_out] = ode15s(arm_dyn, t_span, z0, options);
%             [t_out, z_out] = ode45(arm_dyn, t_span, z0, options);

            % process
            t_out = t_out';
            z_out = z_out';            
        end

        % add measurement noise
        function add_measurement_noise(A, t_span)
            A.measurement_noise_time_ = linspace(t_span(1), t_span(end), A.measurement_noise_size_);
            if A.add_measurement_noise_            
%                 A.measurement_noise_pos_ = 5e-4 * randn(A.n_states/2, A.measurement_noise_size_); % noise profile should try to match actual joint encoders
%                 A.measurement_noise_vel_ = 5e-4 * randn(A.n_states/2, A.measurement_noise_size_);
                A.measurement_noise_pos_ = 1e-4 * randn(A.n_states/2, A.measurement_noise_size_); % noise profile should try to match actual joint encoders
                A.measurement_noise_vel_ = 1e-4 * randn(A.n_states/2, A.measurement_noise_size_);
            else            
                A.measurement_noise_pos_ = zeros(A.n_states/2, A.measurement_noise_size_);
                A.measurement_noise_vel_ = zeros(A.n_states/2, A.measurement_noise_size_);
            end        
        end
        
        %% dynamics
        function zd = dynamics(A, t, z, planner_info)    
            % get actual robot state
            z = z(:);
            q = z(A.joint_state_indices);
            qd = z(A.joint_speed_indices);

            % true dynamics 
            [M, C, g] = A.calculate_dynamics(q, qd, A.params.true);
            
            % add measurement noise if desired
            if A.add_measurement_noise_
                [noise_pos, noise_vel] = match_trajectories(t, A.measurement_noise_time_, A.measurement_noise_pos_, A.measurement_noise_time_, A.measurement_noise_vel_, 'linear'); % linearly interpolate noise
                q_meas = q + noise_pos;
                qd_meas = qd + noise_vel;
            else
                q_meas = q;
                qd_meas = qd;
            end
            z_meas = zeros(size(z));
            z_meas(A.joint_state_indices, 1) = q_meas;
            z_meas(A.joint_speed_indices, 1) = qd_meas;

            u = A.LLC.get_control_inputs(A, t, z_meas, planner_info);

            % update acceleration 
            qdd = M\(u-C*qd-g);

            % preallocate dynamics
            zd = zeros(A.n_states,1) ;

            % compute dynamics
            zd(A.joint_state_indices) = qd(:) ;
            zd(A.joint_speed_indices) = qdd(:) ;
        end

        function [M, C, g] = calculate_dynamics(~, q, qd, params)
            M = rnea_mass(q, params);
            C = rnea_coriolis(q, qd, params);
            g = rnea_gravity(q, params);
        end
        
        %% move
        function move(A,t_move,T_ref,U_ref,Z_ref,planner_info)
            % method: move(t_move,T_ref,U_ref,Z_ref)
            %
            % Moves the agent for the duration t_move using the nominal
            % inputs U_ref and nominal trajectory Z_ref that are indexed by
            % the nominal time T_ref.
            %
            % This method assumes that the input is zero-order hold, and
            % the input corresponding to the last time index is zero; this
            % is why the last old input is discarded when the input is
            % updated. Similarly, this method assumes that the nominal time
            % starts at 0.
            
            A.vdisp('Moving!',5)
            
            % set up default reference trajectory
            if nargin < 5
                Z_ref = [] ;
            end
            
            % get the time, input, and reference trajectory to use for
            % moving the agent
            [T_used,U_used,Z_used] = A.move_setup(t_move,T_ref,U_ref,Z_ref) ;

            % also get reference acceleration:
            qdd_des = [];
            for i = 1:length(T_used)
                [~, ~, qdd_des(:, i)] = planner_info.desired_trajectory{end}(T_used(i));
            end
            
            % get the current state
            zcur = A.state(:,end) ;
            
            switch A.move_mode
                case 'integrator'
                    % call the ode solver to simulate agent
                    [tout,zout] = A.integrator(@(t,z) A.dynamics(t,z,planner_info),...
                                               [0 t_move], zcur) ;
                                           
                    % initialize trajectories to log
                    uout = zeros(A.n_inputs, size(tout, 2));
                    nominal_out = zeros(size(uout));
                    robust_out = zeros(size(uout));
                    disturbance_out = zeros(size(uout));
                    lyap_out = zeros(size(tout));
                    r_out = zeros(A.n_states/2, size(tout, 2));

                    % store approximate inputs at each time:
                    for j = 1:length(tout)
                        t = tout(j);
                        z_meas = zout(:,j);
                        [uout(:, j), nominal_out(:, j), robust_out(:, j),...
                         disturbance_out(:, j), lyap_out(:, j), r_out(:, j)] = ...
                         A.LLC.get_control_inputs(A, t, z_meas, planner_info);
                    end
                case 'direct'
                    % don't call the integrator, just assume the agent
                    % perfectly executes the reference trajectory
                    
                    % get the reference trajectory up to time t_move
                    tout = 0:A.integrator_time_discretization:t_move ;
                    zout = match_trajectories(tout,T_ref,Z_ref) ;
                    uout = zeros(A.n_inputs, size(tout, 2));
                    nominal_out = zeros(size(uout));
                    robust_out = zeros(size(uout));
                    disturbance_out = zeros(size(uout));
                    lyap_out = zeros(size(tout));
                    r_out = zeros(A.n_states/2, size(tout, 2));
                otherwise
                    error('Please set A.move_mode to ''integrator'' or ''direct''!')
            end
            
            A.commit_move_data(tout,zout,T_used,uout,Z_used,nominal_out,robust_out,disturbance_out,lyap_out,r_out,qdd_des) ;

        end
        
        function out = joint_limit_check(A, t_start)
            % create time vector for checking
            t_agent = A.time(end);
            t_check = t_start:A.traj_check_time_discretization:t_agent;

            if isempty(t_check) || t_check(end) ~= t_agent
                t_check = [t_check, t_agent] ;
            end

            % get agent state trajectories interpolated to time
            z_agent = match_trajectories(t_check,A.time,A.state) ;

            % check bound satisfaction
            A.vdisp('Running joint limits check!',3);
            out = false;
            for t_idx = 1:length(t_check)
                q = z_agent(A.joint_state_indices, t_idx);
                qd = z_agent(A.joint_speed_indices, t_idx);
                for i = 1:length(q)
                    if q(i) < A.joint_state_limits(1, i)
                        fprintf('Time %.2f, joint %d position limit exceeded: %.5f vs %.5f \n', t_check(t_idx), i, q(i), A.joint_state_limits(1, i));
                        out = true;
                    end
                    if q(i) > A.joint_state_limits(2, i)
                        fprintf('Time %.2f, joint %d position limit exceeded: %.5f vs %.5f \n', t_check(t_idx), i, q(i), A.joint_state_limits(2, i));
                        out = true;
                    end
                    if qd(i) < A.joint_speed_limits(1, i)
                        fprintf('Time %.2f, joint %d velocity limit exceeded: %.5f vs %.5f \n', t_check(t_idx), i, qd(i), A.joint_speed_limits(1, i));
                        out = true;
                    end
                    if qd(i) > A.joint_speed_limits(2, i)
                        fprintf('Time %.2f, joint %d velocity limit exceeded: %.5f vs %.5f \n', t_check(t_idx), i, qd(i), A.joint_speed_limits(2, i));
                        out = true;
                    end
                end
            end
            
            if ~out
                A.vdisp('No joint limits exceeded', 3);
            end
            
        end
    end    
end

