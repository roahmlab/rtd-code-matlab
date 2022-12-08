classdef RoboticsToolboxArmRobotInfo < handle

    properties
        robot
        dimension
        %n_inputs -> controller
        n_links_and_joints = 7
        
        link_sizes
        link_shapes
        link_masses
        link_inertia_matrices
        
        joint_types
        joint_axes
        joint_locations
        
        kinematic_chain
        
        joint_position_limits
        joint_velocity_limits
        joint_input_limits
        reach_limits
        
        buffer_dist
        params
        link_poly_zonotopes
        %LLC_info -> controller
        
        q_index
    end
    
    methods
        function self = RoboticsToolboxArmRobotInfo(robot, params, options)
            arguments
                robot rigidBodyTree
                params
                options.gravity = [0 0 -9.81];
                options.joint_position_limits = [];
                options.joint_velocity_limits = [];
            end
            % Input/Output format for kinematics and dynamics.
            robot.DataFormat = 'col';
            robot.Gravity = options.gravity;
            self.robot = robot;
            
            self.dimension = 3;
            
            self.n_links_and_joints = params.nominal.num_joints;
            q_index = params.nominal.q_index;
            
            link_shapes = repmat({'cuboid'}, 1, n_links_and_joints);
            [link_poly_zonotopes, link_sizes, temp_link_CAD_data] = create_pz_bounding_boxes(robot);

            % these joint locations are taken in link-centered body-fixed frames
            % imagine a frame at the center of link 1, and at the center of link 2
            % we want to write the position of joint 2 in link 1's frame, and in link 2's frame.
            for i = 1:n_links_and_joints
                if i == 1
                    joint_locations(:, i) = [params.nominal.T0(1:3, end, i);
                                             -link_poly_zonotopes{i}.c];
                else
                    joint_locations(:, i) = [-link_poly_zonotopes{i-1}.c + params.nominal.T0(1:3, end, i);
                                             -link_poly_zonotopes{i}.c];
                end
            end

            % pull joint position limits from rigidBodyTree
            % MATLAB doesn't store velocity/input limits, so pass these in externally for now.
            for i = 1:n_links_and_joints
                if ~strcmp(robot.Bodies{i}.Joint.Type, 'fixed')
                    joint_state_limits(1, q_index(i)) = robot.Bodies{i}.Joint.PositionLimits(1);
                    joint_state_limits(2, q_index(i)) = robot.Bodies{i}.Joint.PositionLimits(2);
                end
            end
       
            % assuming serial kinematic chain!
            kinematic_chain = [0:n_links_and_joints-1; 1:n_links_and_joints];
            self.q_index = q_index
            
        end
        %% property check
        function check_and_fix_properties(self)
            % A.check_and_fix_properties()
            %
            % Go through each of the arm's default properties and make sure
            % they are consisten with the total number of links and joints,
            % and with the dimension of the arm's workspace.
            
            % TODO add back vdisp
            %self.vdisp('Ensuring all arm properties are consistent',3)
            
            % get the actual number of links and joints; the number of
            % links MUST be equal to the number of joints, because every
            % joint is connected to a single predecessor and a single
            % successor link
            N = self.n_links_and_joints ;
            if isempty(N)
                %self.vdisp('Setting A.n_links_and_joints based on link sizes',1)
                N = size(self.link_sizes,2) ;
                self.n_links_and_joints = N ;
            end
            
            % check dimension
            d = self.dimension ;
            if (d ~= 2) && (d ~= 3)
                error(['The arm''s workspace dimension (',num2str(d),...
                    ') is incorrect! Pick 2 or 3.'])
            end
            
            %% links
            %self.vdisp('Checking links',5)
            
            % check dimension and number of links
            [d_l, N_l] = size(self.link_sizes) ;
            
            if N_l ~= N
                error(['The arm does not have the same number of links as ',...
                    'its n_links_and_joints property!'])
            end
            
            if d_l ~= d
                error(['The arm''s links are not of the same dimension (',...
                    num2str(d_l),' as the arm''s dimension property (',...
                    num2str(d),')!'])
            end
            
            % check link shapes
            N_link_shapes = length(self.link_shapes) ;
            if N_link_shapes > N
                %self.vdisp('Removing extra link shapes',6)
                self.link_shapes = self.link_shapes(1:N) ;
            elseif N_link_shapes < N
                %self.vdisp('Setting missing link shapes',6)
                switch d
                    case 2
                        self.link_shapes = [self.link_shapes, repmat({'box'},1,N-N_link_shapes)] ;
                    case 3
                        self.link_shapes = [self.link_shapes, repmat({'cuboid'},1,N-N_link_shapes)] ;
                end
            end
            
            % check link masses
            if isempty(self.link_masses) || length(self.link_masses) ~= N
                %self.vdisp('Setting link masses',6)
                self.link_masses = ones(1,N) ;
            end
            
            % check link inertia matrices
            % see: en.wikipedia.org/wiki/List_of_moments_of_inertia
            if isempty(self.link_inertia_matrices)
                %self.vdisp('Creating link inertia matrices!',6)
                
                J_cell = cell(1,N) ;
                
                for idx = 1:N
                    m = self.link_masses(idx) ;
                    l = self.link_sizes(:,idx) ;                    
                    switch self.link_shapes{idx}
                        case {'box', 'oval'}
                            % treat things as thin boxes in 2-D
                            I_x = 0 ;
                            I_y = 0 ;
                            I_z = (1/12)*m*sum(l.^2) ;                            
                        case 'cuboid'
                            I_x = (1/12)*m*((l(2)+l(3)).^2) ;
                            I_y = (1/12)*m*((l(1)+l(3)).^2) ;
                            I_z = (1/12)*m*((l(1)+l(2)).^2) ;                            
                        case 'ellipsoid'
                            l = l./2 ;
                            I_x = (1/5)*m*((l(2)+l(3)).^2) ;
                            I_y = (1/5)*m*((l(1)+l(3)).^2) ;
                            I_z = (1/5)*m*((l(1)+l(2)).^2) ;
                        case 'cylinder'
                            if l(2) ~= l(3)
                                warning(['The size definition of link ',num2str(idx),...
                                    ' has two mismatched diameters!'])
                            end
                            
                            h = l(1) ;
                            r = l(2)/2 ;
                            
                            I_x = (1/2)*m*r^2 ;
                            I_y = (1/12)*m*(3*r^2 + h) ;
                            I_z = (1/12)*m*(3*r^2 + h) ;
                        otherwise
                            error([self.link_shapes{idx}, 'is an unsupported link shape!'])
                    end
                    J_cell{idx} = diag([I_x, I_y, I_z]) ;                    
                end                
                self.link_inertia_matrices = J_cell ;
            end
            
            %% joints
            %self.vdisp('Checking joints',5)
            
            % check dimension and number of joints
            [d_j_2x, N_j] = size(self.joint_locations) ;
            d_j = d_j_2x / 2 ;
            
            if N_j ~= N
                error(['The arm does not have the same number of joints as ',...
                    'its n_links_and_joints property!'])
            end

            if d_j ~= d
                error(['The arm''s joints are not of the same dimension (',...
                    num2str(d_j),') as the arm''s dimension property (',...
                    num2str(d),')!'])
            end
            
            % set default joint types (all revolute)
            if isempty(self.joint_types) || (length(self.joint_types) ~= N)
                self.joint_types = repmat({'revolute'},1,N) ;
            end
            
            % set default joint axes (note that these axes are in the
            % local coordinate frame of the joint's predecessor link)
            N_joint_axes = size(self.joint_axes,2) ;
            if N_joint_axes > N
                %self.vdisp('Removing extra joint axes',6)
                self.joint_axes = self.joint_axes(:,1:N) ;
            elseif N_joint_axes < N
                %self.vdisp('Setting missing joint axes',6)
                self.joint_axes = [self.joint_axes, repmat([0;0;1],1,N-N_joint_axes)] ;
            end
            
            if isempty(self.kinematic_chain)
                %self.vdisp('Setting arm kinematic chain',6)
                self.kinematic_chain = [0:(N-1) ; 1:N] ;
            end
            
                        
            %% states and state indices
            %self.vdisp('Checking states and state indices',5)
            
            % TODO MOVE TO ControllerComponent
%             if isempty(self.n_inputs)
%                 self.n_inputs = N ;
%                 self.vdisp(['Setting number of inputs to ',num2str(self.n_states)],6)
%             end
            %% physics
            % TODO MOVE TO DYNAMICS
%             if isempty(self.gravity_direction)
%                 %self.vdisp('Setting default gravity direction',9)
%                 switch d
%                     case 2
%                         self.gravity_direction = [0;-1;0] ;
%                     case 3
%                         self.gravity_direction = [0;0;-1] ;
%                 end
%             end
        end
        
        function J = get_joint_locations_from_configuration(A,q)
            % J = A.get_joint_locations_from_configuration(q)
            %
            % Return the joint locations just like A.get_joint_locations,
            % but this is faster since it does not return the rotation
            % matrices or translations of each joint, and can only take in
            % a configuration.
            
            j_vals = q ; % joint angles
            j_locs = A.joint_locations ; % joint locations
            
            % extract dimensions
            n = A.n_links_and_joints ;
            d = A.dimension ;
            
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
                axis_pred = R_pred*A.joint_axes(:,idx) ;
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
        
        function J = get_end_effector_location(A,q)
            % J = A.get_end_effector_location(q)
            %
            % Does what it says! Given a configuration q, this method
            % returns the end effector location.
            
            J = A.get_joint_locations_from_configuration(q) ;
            J = J(:,end) ;
        end
    end
end
