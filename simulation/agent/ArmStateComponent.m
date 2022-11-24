classdef ArmStateComponent < handle
    %ARMSTATECOMPONENT Data and Behavior for State of an Arm
    %   Holds the time-evolving data for the state of the arm along with
    %   functions that act on that data to produce other state-related
    %   data. Forward kinematics is part of this.
    
    properties
        % General information of the robot arm
        arm_info RoboticsToolboxArmRobotInfo
        
        % state space representation
        n_states uint32 = 0
        state = []
        time (1,:) = []
        
        % indexes for state space
        position_indices (1,:) uint32 = []
        velocity_indices (1,:) uint32 = []
    end
    properties (Dependent)
        position
        velocity
    end
    
    methods
        function self = ArmStateComponent(arm_info)
            self.arm_info = arm_info;
            self.n_states = 2 * arm_info.n_links_and_joints;
            self.position_indices = 1:2:self.n_states;
            self.velocity_indices = 2:2:self.n_states;
            
            % initialize
            self.reset();
        end
        
        function reset(self,state,joint_speeds)
            % TODO Add back vdisp
            %self.vdisp('Resetting states',3) ;
            
            % reset to zero by default
            self.state = zeros(self.n_states,1) ;
            % TODO Move to controller
            %self.reference_state = zeros(self.n_states,1) ;
            %self.reference_acceleration = zeros(self.n_states/2,1) ;
            
            if nargin > 1
                % changed to be more general
                if length(state) == self.n_states / 2
                    % fill in joint positions if they are provided
                    %self.vdisp('Using provided joint positions',6)
                    self.state(self.position_indices) = state ;
                    %self.reference_state(self.position_indices) = state ;
                    
                    if nargin > 2
                        % fill in joint speeds if they are provided
                        %self.vdisp('Using provided joint speeds',6)
                        self.state(self.velocity_indices) = joint_speeds ;
                        %self.reference_state(self.velocity_indices) = joint_speeds ;
                    end
                elseif length(state) == self.n_states
                    % fill in full position and speed state if provided
                    %self.vdisp('Using provided full state',6)
                    self.state = state ;
                    %self.reference_state = state ;
                else
                    error('Input has incorrect number of states!')
                end
            end
            
            %self.vdisp('Resetting time and inputs',3)
            self.time = 0 ;
            
            % TODO Move to controller
            %self.input = zeros(self.n_inputs,1) ;
            %self.input_time = 0 ;
            %
            % reset LLC
            %if isa(self.LLC,'arm_PID_LLC')
            %    self.vdisp('Resetting low-level controller integrator error.',3)
            %    self.LLC.position_error_state = zeros(length(self.joint_state_indices),1) ;
            %end
        end
        
        % random state generation TODO EVALUATE
        function varargout = create_random_state(self)
            % [z,u] = A.create_random_state()
            % [q,qd,u] = A.create_random_state()
            %
            % Create a random state z and random input u, given the arm's
            % state, speed, and input limits. If three output args are
            % specified, then return the config q, joint speeds qd, and
            % input u.
            
            % set any joint limits that are +Inf to pi and -Inf to -pi
            state_lims = self.arm_info.joint_state_limits ;
            joint_limit_infs = isinf(state_lims) ;
            state_lims(1,joint_limit_infs(1,:)) = -pi ;
            state_lims(2,joint_limit_infs(2,:)) = +pi ;
            
            % make random state
            q = rand_range(state_lims(1,:),state_lims(2,:))' ;
            qd = rand_range(self.arm_info.joint_speed_limits(1,:),self.arm_info.joint_speed_limits(2,:))' ;
            
            % make random input
            % TODO MOVE TO CONTROLLER
            %u = rand_range(self.joint_input_limits(1,:),self.joint_input_limits(2,:))' ;
            
            % TODO UPDATE BEHAVIOR
            switch nargout
                case 1
                    varargout = {q} ;
                case 2                    
                    z = nan(self.n_states,1) ;
                    z(self.position_indices) = q ;
                    z(self.velocity_indices) = qd ;
                    
                    varargout = {z, u} ;
                case 3
                    varargout = {q, qd, u} ;
            end
        end
        
        function commit_state_data(self,T_state,Z_state)
            % method: commit_move_data(T_state,Z_state,T_input,U_input,Z_input)
            %
            % After moving the agent, commit the new state and input
            % trajectories, and associated time vectors, to the agent's
            % state, time, input, and input_time properties.
            
            % update the state, time, input, and input time
            self.state = [self.state, Z_state(:,2:end)] ;
            self.time = [self.time, self.time(end) + T_state(2:end)] ;
            % TODO MOVE TO RESPECTIVE PLACES AND DASH WHEN DONE
%             self.input_time = [self.input_time, self.input_time(end) + T_used(2:end)] ;
%             self.input = [self.input, U_used(:,1:end-1)] ;
%             self.nominal_input = [self.nominal_input, U_nominal_used(:, 1:end-1)];
%             self.robust_input = [self.robust_input, U_robust_used(:, 1:end-1)];
%             self.disturbance = [self.disturbance, U_disturbance_used(:, 1:end-1)];
%             self.reference_state = [self.reference_state, Z_used(:, 2:end)];
%             self.reference_acceleration = [self.reference_acceleration, qdd_des(:, 2:end)];
%             self.lyapunov = [self.lyapunov, V_used(1, 1:end-1)];
%             self.r = [self.r, r_used(:, 1:end-1)];
        end
        
        %% TODO review
        function [R,T,J] = get_link_rotations_and_translations(A,time_or_config,cad_flag)
            % [R,T] = A.get_link_rotations_and_translations(time)
            % [R,T] = A.get_link_rotations_and_translations(configuration)
            % [R,T,J] = A.get_link_rotations_and_translations(t_or_q)
            %
            % Compute the rotation and translation of all links in the
            % global (baselink) frame at the given time. If no time is
            % given, then it defaults to 0.
            %
            % The optional third output is the joint locations in 2- or 3-D
            % space, which is also output by A.get_joint_locations(t_or_q).
            %
            % Updated by Patrick on 20220425 to deal with fixed joints
            
            if nargin < 2
                time_or_config = 0 ;
            end
            
            if nargin < 3
                cad_flag = false;
            end
            
            % get joint data
            if size(time_or_config,1) == 1
                t = time_or_config ;
                if t > A.time(end)
                    t = A.time(end) ;
                    warning(['Invalid time entered! Using agent''s final ',...
                        'time t = ',num2str(t),' instead.'])
                end
                
                % interpolate the state for the corresponding time
                z = match_trajectories(t,A.time,A.state) ;
                j_vals = z(1:2:end) ; % joint values
            else
                % assume a configuration was put in
                q = time_or_config ;
                
                if length(q) == A.n_states
                    q = q(1:2:end) ;
                elseif length(q) ~= A.n_states/2
                    error('Please provide either a time or a joint configuration.')
                end
                j_vals = q ;
            end
            
            if ~cad_flag
                j_locs = A.joint_locations ; % joint locations
            else
                j_locs = A.joint_locations_CAD ; % joint locations 
            end
            
            % extract dimensions
            n = A.n_links_and_joints ;
            d = A.dimension ;
            
            % allocate cell arrays for the rotations and translations
            R = mat2cell(repmat(eye(d),1,n),d,d*ones(1,n)) ;
            T = mat2cell(repmat(zeros(d,1),1,n),d,ones(1,n)) ;
            
            % allocate array for the joint locations
            J = nan(d,n) ;
            
            % move through the kinematic chain and get the rotations and
            % translation of each link
            for idx = 1:n
                k_idx = A.kinematic_chain(:,idx) ;
                p_idx = k_idx(1) ;
                s_idx = k_idx(2) ;
                
                % get the rotation and translation of the predecessor and
                % successor links; we assume the baselink is always rotated
                % to an angle of 0 and with a translation of 0
                if p_idx == 0
                    R_pred = eye(d) ;
                    T_pred = zeros(d,1) ;
                else
                    R_pred = R{p_idx} ;
                    T_pred = T{p_idx} ;
                end
                
                % get the location of the current joint
                j_loc = j_locs(:,idx) ;
                
                % compute link rotation
                switch A.joint_types{idx}
                    case 'revolute'
                        % get value of current joint
                        j_idx = j_vals(A.q_index(idx)) ;
                        if d == 3
                            % rotation matrix of current link
                            axis_pred = A.robot.Bodies{idx}.Joint.JointToParentTransform(1:3, 1:3)*R_pred*A.joint_axes(:,idx) ;
                            R_succ = axis_angle_to_rotation_matrix_3D([axis_pred', j_idx])*A.robot.Bodies{idx}.Joint.JointToParentTransform(1:3, 1:3)*R_pred ;
                        else
                            % rotation matrix of current link
                            R_succ = rotation_matrix_2D(j_idx)*R_pred ;
                        end
                        
                        % create translation
                        T_succ = T_pred + R_pred*j_loc(1:d) - R_succ*j_loc(d+1:end) ;
                    case 'prismatic'
                        % R_succ = R_pred ;
                        error('Prismatic joints are not yet supported!')
                    case 'fixed'
                        if d == 3
                            R_succ = A.robot.Bodies{idx}.Joint.JointToParentTransform(1:3, 1:3)*R_pred ;
                        else
                            % rotation matrix of current link assumed same as predecessor
                            R_succ = R_pred ;
                        end
                        % create translation
                        T_succ = T_pred + R_pred*j_loc(1:d) - R_succ*j_loc(d+1:end) ;
                    otherwise
                        error('Invalid joint type!')
                end
                
                % fill in rotation and translation cells
                R{s_idx} = R_succ ;
                T{s_idx} = T_succ ;
                
                % fill in the joint location
                j_loc_local = j_locs((d+1):end,idx) ;
                J(:,idx) = -R_succ*j_loc_local + T_succ ;
            end
        end
        
        function  J = get_joint_locations(A,times_or_configs)
            % J = A.get_joint_locations(times_or_configs)
            %
            % Return the joint locations in 2-D or 3-D space.
            %
            % If the input is a single time t \in \R, or a single
            % configuration q \in Q, the output is a d-by-n array, where
            % n = A.n_links_and_joints and d = A.dimension.
            %
            % If the input is a 1-by-N vector of times or an n-by-N vector
            % of configurations, the output is a 1-by-N cell array where
            % each entry is the d-by-n array of joint locations for the
            % corresponding time or configuration.
            
            N = size(times_or_configs,2) ;
            if N == 1
                [~,~,J] = A.get_link_rotations_and_translations(times_or_configs) ;
            else
                J = cell(1,N) ;
                n = size(times_or_configs,1) ;
                
                if n == 1
                    % for the case of multiple times, iterate through the
                    % list and get the joint locations for each time
                    for idx = 1:N
                        [~,~,J_idx] = A.get_link_rotations_and_translations(times_or_configs(:,idx)) ;
                        J{idx} = J_idx ;
                    end
                else
                    % for the case of multiple configurations, make a cell
                    % array of the configurations and use cellfun like a
                    % heckin' matlab ninja
                    Q = mat2cell(times_or_configs, n, ones(1,N)) ;
                    J = cellfun(@(q) A.get_joint_locations_from_configuration(q),Q,'UniformOutput',false) ;
                end
            end
        end
        
        function [R,T,J] = forward_kinematics(A,time_or_config)
            % [R,T,J] = A.forward_kinematics(time_or_config)
            %
            % Given a time or configuration, return the link rotation
            % and translation arrays R and T, and the joint locations J.
            
            [R,T,J] = A.get_link_rotations_and_translations(time_or_config) ;
        end
        
        function ee_pos = forward_kinematics_end_effector(A,time_or_config)
            % ee_pos = forward_kinematics_end_effector(A,time_or_config)
            %
            % Return the position of the end effector as a location in
            % workspace (either 2-D or 3-D), given an input configuration;
            % the output is a vector of length A.dimension.
            [~,~,J] = A.get_link_rotations_and_translations(time_or_config) ;
            ee_pos = J(:,end) ;
        end
        
        function position = get.position(self)
            position = self.state(self.position_indices,:);
        end
        function velocity = get.velocity(self)
            velocity = self.state(self.velocity_indices,:);
        end
    end
end

