classdef ArmKinematicsComponent < handle
    % a collection of useful functions for arm robot kinematics
    properties
        arm_info
        arm_state
    end
    
    methods
        function self = ArmKinematicsComponent(arm_info, arm_state_component)
            arguments
                arm_info
                arm_state_component
            end
            self.arm_info = arm_info;
            self.arm_state = arm_state_component;
        end
        %% TODO review
        function [R,T,J] = get_link_rotations_and_translations(self,time_or_config, cad_flag)
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
                if t > self.arm_state.time(end)
                    t = self.arm_state.time(end) ;
                    warning(['Invalid time entered! Using agent''s final ',...
                        'time t = ',num2str(t),' instead.'])
                end
                
                % interpolate the state for the corresponding time
                z = match_trajectories(t,self.arm_state.time,self.arm_state.state) ;
                j_vals = z(1:2:end) ; % joint values
            else
                % assume a configuration was put in
                q = time_or_config ;
                
                if length(q) == self.arm_state.n_states
                    q = q(1:2:end) ;
                elseif length(q) ~= self.arm_state.n_states/2
                    error('Please provide either a time or a joint configuration.')
                end
                j_vals = q ;
            end
            
            if ~cad_flag
                j_locs = self.arm_info.joint_locations ; % joint locations
            else
                % We just copy it from the cad
                j_locs = 0*self.arm_info.joint_locations;
                for i=1:self.arm_info.n_links_and_joints
                    j_locs(:, i) = [self.arm_info.params.nominal.T0(1:3, end, i);
                                                 zeros(3, 1)];
                end
                % j_locs = A.joint_locations_CAD ; % joint locations 
            end
            
            % extract dimensions
            n = self.arm_info.n_links_and_joints ;
            d = self.arm_info.dimension ;
            
            % allocate cell arrays for the rotations and translations
            R = mat2cell(repmat(eye(d),1,n),d,d*ones(1,n)) ;
            T = mat2cell(repmat(zeros(d,1),1,n),d,ones(1,n)) ;
            
            % allocate array for the joint locations
            J = nan(d,n) ;
            
            % move through the kinematic chain and get the rotations and
            % translation of each link
            for idx = 1:n
                k_idx = self.arm_info.kinematic_chain(:,idx) ;
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
                switch self.arm_info.joint_types{idx}
                    case 'revolute'
                        % get value of current joint
                        j_idx = j_vals(self.arm_info.q_index(idx)) ;
                        if d == 3
                            % rotation matrix of current link
                            axis_pred = self.arm_info.robot.Bodies{idx}.Joint.JointToParentTransform(1:3, 1:3)*R_pred*self.arm_info.joint_axes(:,idx) ;
                            R_succ = axis_angle_to_rotation_matrix_3D([axis_pred', j_idx])*self.arm_info.robot.Bodies{idx}.Joint.JointToParentTransform(1:3, 1:3)*R_pred ;
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
                            R_succ = self.arm_info.robot.Bodies{idx}.Joint.JointToParentTransform(1:3, 1:3)*R_pred ;
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
        
        function  J = get_joint_locations(self,times_or_configs)
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
                [~,~,J] = self.get_link_rotations_and_translations(times_or_configs) ;
            else
                J = cell(1,N) ;
                n = size(times_or_configs,1) ;
                
                if n == 1
                    % for the case of multiple times, iterate through the
                    % list and get the joint locations for each time
                    for idx = 1:N
                        [~,~,J_idx] = self.get_link_rotations_and_translations(times_or_configs(:,idx)) ;
                        J{idx} = J_idx ;
                    end
                else
                    % for the case of multiple configurations, make a cell
                    % array of the configurations and use cellfun like a
                    % heckin' matlab ninja
                    Q = mat2cell(times_or_configs, n, ones(1,N)) ;
                    J = cellfun(@(q) self.get_joint_locations_from_configuration(q),Q,'UniformOutput',false) ;
                end
            end
        end
        
        function J = get_joint_locations_from_configuration(self,q)
            % J = A.get_joint_locations_from_configuration(q)
            %
            % Return the joint locations just like A.get_joint_locations,
            % but this is faster since it does not return the rotation
            % matrices or translations of each joint, and can only take in
            % a configuration.
            
            j_vals = q ; % joint angles
            j_locs = self.arm_info.joint_locations ; % joint locations
            
            % extract dimensions
            n = self.arm_info.n_links_and_joints ;
            d = self.arm_info.dimension ;
            
            % set up translations and rotations
            R_pred = eye(d) ;
            T_pred = zeros(d,1) ;
            
            % allocate array for the joint locations
            J = nan(d,n) ;
            
            if isa(q,'sym')
                J = sym(J) ;
            end
            
            % move through the kinematic chain and get the rotations and
            % translation of each link
            for idx = 1:n
                % get the value and location of the current joint
                j_idx = j_vals(idx) ;
                j_loc = j_locs(:,idx) ;
                
                % rotation matrix of current joint
                axis_pred = R_pred*self.arm_info.joint_axes(:,idx) ;
                R_succ = axis_angle_to_rotation_matrix_3D([axis_pred', j_idx])*R_pred ;
                
                % create translation
                T_succ = T_pred + R_pred*j_loc(1:d) - R_succ*j_loc(d+1:end) ;
                
                % fill in the joint location
                j_loc_local = j_locs((d+1):end,idx) ;
                J(:,idx) = -R_succ*j_loc_local + T_succ ;
                
                % update predecessors for next iteration
                R_pred = R_succ ;
                T_pred = T_succ ;
            end
        end
        
        function J = get_end_effector_location(self,q)
            % J = A.get_end_effector_location(q)
            %
            % Does what it says! Given a configuration q, this method
            % returns the end effector location.
            
            J = self.get_joint_locations_from_configuration(q) ;
            J = J(:,end) ;
        end
        
        function [R,T,J] = forward_kinematics(self,time_or_config)
            % [R,T,J] = A.forward_kinematics(time_or_config)
            %
            % Given a time or configuration, return the link rotation
            % and translation arrays R and T, and the joint locations J.
            
            [R,T,J] = self.get_link_rotations_and_translations(time_or_config) ;
        end
        
        function ee_pos = forward_kinematics_end_effector(self,time_or_config)
            % ee_pos = forward_kinematics_end_effector(A,time_or_config)
            %
            % Return the position of the end effector as a location in
            % workspace (either 2-D or 3-D), given an input configuration;
            % the output is a vector of length A.dimension.
            [~,~,J] = self.get_link_rotations_and_translations(time_or_config) ;
            ee_pos = J(:,end) ;
        end
        
        
        %% inverse kinematics
        function [q,exitflag] = inverse_kinematics(self,J,q0)
            % q = A.inverse_kinematics(J)
            % q = A.inverse_kinematics(J,q0)
            % [q,exitflag] = A.inverse_kinematics(...)
            %
            % Given a desired end-effector location, or locations of all
            % the joints, J, as an A.dimension-by-(1 or A.n_links...)
            % array, attempt to find a configuration q that reaches that
            % location. This uses fmincon for nonlinear optimization of the
            % Euclidean distances squared to each joint location as the
            % cost function. The second (optional) argument is an initial
            % guess for the nonlinear solver.
            
            if nargin < 3
                q0 = self.arm_state.state(self.arm_state.position_indices,end) ;
            end
            
            if size(J,2) > 1
                [q,exitflag] = self.inverse_kinematics_joint_locations(J,q0) ;
            else
                [q,exitflag] = self.inverse_kinematics_end_effector(J,q0) ;
            end
        end
        
        function [q,exitflag] = inverse_kinematics_joint_locations(self,J,q0)
            % q = A.inverse_kinematics_joint_locations(J)
            % q = A.inverse_kinematics_joint_locations(J,q0)
            %
            % Given joint locations J as a d-by-n array, return the
            % static configuration q as an n-by-1 vector where n is the
            % number of joints of the arm. This uses nonlinear optimization
            % (fmincon) to find the joint configuration.
            %
            % The optional second input is an initial guess for the
            % nonlinear least squares solver. If it is not returned, the
            % arm uses its last state (the column vector A.state(:,end)) as
            % the initial guess.
            
            % set up the initial config
            if nargin < 3
                q0 = self.arm_state.state(self.arm_state.position_indices,end) ;
            end
            
            % create the least-squares function to solve for the config
            n = self.arm_info.n_links_and_joints ;
            d = self.arm_info.dimension ;
            opt_fun = @(x) sum(vecnorm(reshape(self.get_joint_locations(x),n*d,1) - J)) ;
            
            % create bounds on the solution
            lb = self.arm_info.joint_position_limits(1,:)' ;
            ub = self.arm_info.joint_position_limits(2,:)' ;
            
            % set options
            options = optimoptions('fmincon') ;
            if A.verbose < 2
                options.Display = 'off' ;
            end
            
            % optimize!
            [q,~,exitflag] = fmincon(opt_fun,q0,[],[],[],[],lb,ub,[],options) ;
        end
        
        function [q,exitflag] = inverse_kinematics_end_effector(self,J,q0)
            % q = A.inverse_kinematics_end_effector(J,q0)
            %
            % Given an end-effector location J \in \R^d where d is the
            % dimension of the agent (2 or 3), find the configuration q
            % that gets the arm to that end effector location (or fail
            % trying! aaaah!)
            
            % set up the initial guess
            if nargin < 3
                q0 = self.arm_state.state(self.position_indices,end) ;
            end
            
            % create the function to solve for the config
            opt_fun = @(x) sum((self.get_end_effector_location(x) - J(:)).^2) ;
            
            % create bounds on the solution
            lb = self.arm_info.joint_position_limits(1,:)' ;
            ub = self.arm_info.joint_position_limits(2,:)' ;
            
            % set options
            options = optimoptions('fmincon') ;
            if A.verbose < 2
                options.Display = 'off' ;
            end
            
            % optimize!
            [q,~,exitflag] = fmincon(opt_fun,q0,[],[],[],[],lb,ub,[],options) ;
        end
    end
end