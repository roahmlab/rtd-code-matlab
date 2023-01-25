classdef ArmourAgentInfo < rtd.entity.components.BaseInfoComponent & rtd.util.mixins.Options & handle
    
    % Leftover Old Dependencies
    % load_robot_params by extension -> make it a class
    % create_pz_bounding_boxes
    % 
    % Notes
    % links should become an independently validatable class
    % joints should as well, or be a part of the links one
    
    properties
        % Parameters of the robot
        robot rigidBodyTree = rigidBodyTree.empty()
        params struct % Consider converting to a class or embedding
        
        % This is required to not change in this case
        dimension = 3
        
        % Useful parameters for our links and joints
        n_links_and_joints(1,1) uint8
        num_q(1,1) uint8
        body_joint_index uint8
        
        % Struct array with size, shape, mass?, and inertia matrix? information
        % and poly zonotope
        links struct
        
        % Struct array with type, axes, and location for the joints
        % and position, velocity, and torque limits
        joints struct
        
        % ???
        kinematic_chain
        reach_limits
        buffer_dist
        
        % Used for finding the ultimate bound (mass matrix)
        M_min_eigenvalue double
    end
    
    methods (Static)
        function options = defaultoptions()
            % Configurable options for this component
            options.M_min_eigenvalue = 0.002; % arbitrary value
            options.gravity = [0 0 -9.81];
            options.joint_velocity_limits = [];
            options.joint_torque_limits = [];
            options.buffer_dist = 0;
        end
    end
    
    methods
        function self = ArmourAgentInfo(robot, params, optionsStruct, options)
            arguments
                robot rigidBodyTree
                params
                optionsStruct struct = struct()
                options.M_min_eigenvalue
                options.gravity
                options.joint_velocity_limits
                options.joint_torque_limits
                options.buffer_dist
            end
            self.mergeoptions(optionsStruct, options);
            
            % we need the robot to have the column dataformat to work with
            % our code, so set that. Gravity is an option so set that
            % elsewhere
            self.robot = copy(robot);
            self.robot.DataFormat = 'col';
            self.params = params;
            
            % initialize
            self.reset();
        end
        
        % We set this here, but is it really necessary?
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.M_min_eigenvalue
                options.gravity
                options.joint_velocity_limits
                options.joint_torque_limits
                options.buffer_dist
            end
            options = self.mergeoptions(optionsStruct, options);
            if isempty(options.joint_velocity_limits)
                error("Must pass in joint_velocity_limits externally!")
            end
            if isempty(options.joint_torque_limits)
                error("Must pass in joint_torque_limits externally!")
            end
            
            
            % Fill in our other dependent parameters
            self.robot.Gravity = options.gravity;
            self.n_links_and_joints = self.params.nominal.num_joints;
            self.num_q = self.params.nominal.num_q;
            self.body_joint_index = self.params.nominal.q_index;

            
            % Flesh out the links from the robot
            link_shapes = repmat({'cuboid'}, 1, self.n_links_and_joints);
            [link_poly_zonotopes, link_sizes, ~] = create_pz_bounding_boxes(self.robot);
            link_sizes = num2cell(link_sizes,1);

            % Store to the class links struct
            self.links = repmat(struct, 1, self.n_links_and_joints);
            [self.links.shape] = link_shapes{:};
            [self.links.size] = link_sizes{:};
            % Do we need link_masses?
            % Do we need link_inertia_matrices?
            % It looks like these are folded into the params
            [self.links.poly_zonotope] = link_poly_zonotopes{:};
            
            
            % Flesh out the joints from the robot
            % these joint locations are taken in link-centered body-fixed frames
            % imagine a frame at the center of link 1, and at the center of link 2
            % we want to write the position of joint 2 in link 1's frame, and in link 2's frame.
            joint_locations(1) = {[ self.params.nominal.T0(1:3, end, 1);
                                    -self.links(1).poly_zonotope.c ]};
            for i = 2:self.n_links_and_joints
                joint_locations(i) = {[-self.links(i-1).poly_zonotope.c + self.params.nominal.T0(1:3, end, i);
                                         -self.links(i).poly_zonotope.c]};
            end
            % pull joint position limits from rigidBodyTree
            % MATLAB doesn't store velocity/input limits, so pass these in externally for now.
            for i = 1:self.n_links_and_joints
                if ~strcmp(self.robot.Bodies{i}.Joint.Type, 'fixed')
                    joint_position_limits(self.body_joint_index(i)) = {self.robot.Bodies{i}.Joint.PositionLimits(:)};
                end
            end
            joint_axes = num2cell(self.params.nominal.joint_axes, 1);
            joint_velocity_limits = num2cell(options.joint_velocity_limits, 1);
            joint_torque_limits = num2cell(options.joint_torque_limits, 1);
            
            % Store to the class joints struct
            self.joints = repmat(struct, 1, self.n_links_and_joints);
            [self.joints.type] = self.params.nominal.joint_types{:};
            [self.joints.axes] = joint_axes{:};
            [self.joints.location] = joint_locations{:};
            % This handling of fixed and non-fixed joints can be a lot
            % cleaner by making this into it's own class
            [self.joints(1:length(joint_position_limits)).position_limits] = joint_position_limits{:};
            [self.joints(1:length(joint_velocity_limits)).velocity_limits] = joint_velocity_limits{:};
            [self.joints(1:length(joint_torque_limits)).torque_limits] = joint_torque_limits{:};
            
            
            % set our M_min_eigenvalue
            self.M_min_eigenvalue = options.M_min_eigenvalue;
            
            
            % The ???? section
            % assuming serial kinematic chain!
            self.kinematic_chain = [0:self.n_links_and_joints-1; 1:self.n_links_and_joints];
            
            % figure out the maximum length and reach of the arm
            % based on axis limits
            % https://www.mathworks.com/help/matlab/ref/axis.html#buk989s-1-limits
            % Also used for plotting
            link_sizes = [self.links.size];
            L = sum(link_sizes(1,:));
            % Compute limits
            lims = [-0.5*L, L, -L, L, -L, 0.75*L] ;
            % in case base of robot is not at [0;0;0]:
            joint1loc = self.joints(1).location;
            lims = lims + [joint1loc(1)*ones(1, 2), joint1loc(2)*ones(1, 2), joint1loc(3)*ones(1, 2)];
            % make z = 0 the ground
            lims(5) = 0;
            self.reach_limits = lims;
            
            % ????
            self.buffer_dist = options.buffer_dist;
        end
    end
end