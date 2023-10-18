classdef WaitrAgentState < armour.agent.ArmourAgentState
    % Wrapper for adding force checks
    
    methods
        function random_init(self, options)
            arguments
                self
                options.x_pos_range = []
                options.y_pos_range = []
                options.z_pos_range = []
                options.vel_range = []
                options.random_position = true
                options.random_velocity = false
                options.save_to_options = false
            end

            x_pos_range = options.x_pos_range;
            y_pos_range = options.y_pos_range;
            z_pos_range = options.z_pos_range;
            vel_range = options.vel_range;

            if isempty(x_pos_range)
                x_pos_range = [-0.4,0.6];
            end
            if isempty(y_pos_range)
                y_pos_range = [-0.5,0.5];
            end
            if isempty(z_pos_range)
                z_pos_range = [0.2,0.6];
            end
            if isempty(vel_range)
                vel_range = [self.entity_info.joints(1:self.entity_info.num_q).velocity_limits];
            end

            ik = robotics.InverseKinematics('RigidBodyTree',self.entity_info.robot);
            
            weights = [0.25 0.25 0.25 1 1 1];
            initialguess = homeConfiguration(self.entity_info.robot);

            % reset
            self.state = zeros(self.n_states,1);
            self.time = 0;
            self.step_start_idxs = 0;
    
            % Make the random configuration
            if options.random_position
                pitch = 1;
                roll = 1;
                while abs(pitch) > 0.001 || abs(roll) > 0.001
                    % choose random x,y,z components for location of 10th joint
                    % in taskspace
                    rand_x = rtd.random.deprecation.rand_range(x_pos_range(1),x_pos_range(2));
                    rand_y = rtd.random.deprecation.rand_range(y_pos_range(1),y_pos_range(2));
                    rand_z = rtd.random.deprecation.rand_range(z_pos_range(1),z_pos_range(2));

                    % solving for a config other than home config
                    new_config = ik('cube_link',[1 0 0 rand_x; 0 1 0 rand_y; 0 0 1 rand_z; 0 0 0 1],weights,initialguess);
                    self.state(self.position_indices) = new_config;
    
                    T = self.entity_info.robot.getTransform(new_config,'cube_link');
%                     yaw=atan2(T(2,1),T(1,1));
                    pitch=atan2(-T(3,1),sqrt(T(3,2)^2+T(3,3)^2));
                    roll=atan2(T(3,2),T(3,3));
                end
            end
            if options.random_velocity
                self.state(self.velocity_indices) = rtd.random.deprecation.rand_range(vel_range(1,:),vel_range(2,:))';
            end

            % Take these initials and merge them in again.
            if options.save_to_options
                merge.initial_position = self.position;
                merge.initial_velocity = self.velocity;
                self.mergeoptions(merge);
            end
        end
    end
end