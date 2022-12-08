classdef ArmStateComponent < handle & NamedClass
    %ARMSTATECOMPONENT Data and Behavior for State of an Arm
    %   Holds the time-evolving data for the state of the arm along with
    %   functions that act on that data to produce other state-related
    %   data. Forward kinematics is part of this.
    
    properties
        % General information of the robot arm
        arm_info RoboticsToolboxArmRobotInfo
        
        % state space representation
        n_states uint32 = 0
        state double = []
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
        
        function state = getState(self, time)
            arguments
                self
                time = -1
            end
            state = ArmRobotState();
            if time < 0
                state.time = self.time(end);
                state.q = self.position(end);
                state.q_dot = self.velocity(end);
            else
                state.time = time;
                temp_state = interp1(self.time, self.state.', time);
                state.q = temp_state(self.position_indices);
                state.q_dot = temp_state(self.velocity_indices);
            end
        end
        
        function reset(self,state,joint_speeds)
            self.vdisp('Resetting states',3) ;
            % reset to zero by default
            self.state = zeros(self.n_states,1) ;
            % Add state
            if nargin > 1
                % changed to be more general
                if length(state) == self.n_states / 2
                    % fill in joint positions if they are provided
                    self.vdisp('Using provided joint positions',6)
                    self.state(self.position_indices) = state ;
                    
                    if nargin > 2
                        % fill in joint speeds if they are provided
                        self.vdisp('Using provided joint speeds',6)
                        self.state(self.velocity_indices) = joint_speeds ;
                    end
                elseif length(state) == self.n_states
                    % fill in full position and speed state if provided
                    self.vdisp('Using provided full state',6)
                    self.state = state ;
                else
                    error('Input has incorrect number of states!')
                end
            end
            self.vdisp('Resetting time and inputs',3)
            self.time = 0 ;
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
        
        function position = get.position(self)
            position = self.state(self.position_indices,:);
        end
        function velocity = get.velocity(self)
            velocity = self.state(self.velocity_indices,:);
        end
    end
end

