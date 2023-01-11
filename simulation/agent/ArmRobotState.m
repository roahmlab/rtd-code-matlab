classdef ArmRobotState < EntityState
    % ArmRobotState
    % Information on the atomic state of the robot at a given point of
    % time. Each hard instance of this (unique object, not seperate
    % handles to the same underlying object) will have a unique uuid.
    properties 
        position_indices (1,:) uint32
        velocity_indices (1,:) uint32
    end
    properties (Dependent)
        q
        q_dot
        position
        velocity
    end

    methods
        function self = ArmRobotState(position_indices, velocity_indices)
            self.position_indices = position_indices;
            self.velocity_indices = velocity_indices;
        end
            

        function position = get.position(self)
            position = self.state(self.position_indices);
        end

        function velocity = get.velocity(self)
            velocity = self.state(self.velocity_indices);
        end

        % Compat
        function q = get.q(self)
            q = self.position;
        end
        
        function q_dot = get.q_dot(self)
            q_dot = self.velocity;
        end
    end
end