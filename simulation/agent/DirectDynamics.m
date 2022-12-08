classdef DirectDynamics < handle
    % DirectDynamics
    % Must be paired with any kind of OpenLoopController and TrajectoryController
    
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
        time = []
        inputs = []
    end
    
    methods
        function self = DirectDynamics(arm_info, arm_state_component, controller_component, options)
            arguments
                arm_info
                arm_state_component
                controller_component
                options.time_discretization = 0.01;
            end
            self.arm_info = arm_info;
            self.arm_state = arm_state_component;
            self.controller = controller_component;
            self.time_discretization = options.integrator_time_discretization;
        end
        
        function move(self, t_move)
            % Move for a duration of time specified by t_move
            T = 0:self.time_discretization:t_move;
            
            % Get the states to passthrough from the controller
            states, control_inputs = self.controller.getControlInputs(T);
            
            % Commit it
            self.arm_state.commit_state_data(self, T, states);
            self.commit_input_data(T, control_inputs);
        end
        
        function commit_input_data(self, T, inputs)
            self.time = [self.time, self.time(end) + T(2:end)];
            self.inputs = [self.inputs, inputs(:,2:end)];
        end
    end
end
