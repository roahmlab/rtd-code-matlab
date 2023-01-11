classdef ArmRobotTrajectoryState < EntityState
    % ArmRobotState
    % Information on the atomic state of the robot at a given point of
    % time. Each hard instance of this (unique object, not seperate
    % handles to the same underlying object) will have a unique uuid.
    properties 
        position_indices (1,:) uint32
        velocity_indices (1,:) uint32
        acceleration_indices (1,:) uint32
    end
    properties (Dependent)
        q
        q_dot
        q_ddot
        q_des
        q_dot_des
        q_ddot_des
        position
        velocity
        acceleration
    end

    methods
        function self = ArmRobotTrajectoryState(position_indices, velocity_indices, acceleration_indices)
            self.position_indices = position_indices;
            self.velocity_indices = velocity_indices;
            self.acceleration_indices = acceleration_indices;
        end
            

        function position = get.position(self)
            position = self.state(self.position_indices);
        end

        function velocity = get.velocity(self)
            velocity = self.state(self.velocity_indices);
        end

        function acceleration = get.acceleration(self)
            acceleration = self.state(self.acceleration_indices);
        end

        % Compat
        function q = get.q_des(self)
            q = self.position;
        end
        
        function q_dot = get.q_dot_des(self)
            q_dot = self.velocity;
        end

        function q_ddot = get.q_ddot_des(self)
            q_ddot = self.acceleration;
        end

        function q = get.q(self)
            q = self.position;
        end
        
        function q_dot = get.q_dot(self)
            q_dot = self.velocity;
        end

        function q_ddot = get.q_ddot(self)
            q_ddot = self.acceleration;
        end
    end
end