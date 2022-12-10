classdef ArmourAgentState < EntityState & NamedClass & OptionsClass & handle
    
    % Inherited properties that must be defined
    properties
        % General information of the robot arm
        robot_info = ArmourAgentInfo.empty()
        
        % state space representation
        n_states = 0
        state double = []
        time = []
    end
    
    % Extra properties we define
    properties
        % indexes for state space
        position_indices (1,:) uint32 = []
        velocity_indices (1,:) uint32 = []
    end
    % Dynamics properties to make life easier
    properties (Dependent)
        position
        velocity
    end
    
    methods (Static)
        function options = defaultoptions()
            options.verboseLevel = LogLevel.INFO;
            options.name = '';
        end
    end
    
    methods
        function self = ArmourAgentState(arm_info, optionsStruct, options)
            arguments
                arm_info ArmourAgentInfo
                optionsStruct struct = struct()
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct, options);
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            % Setup
            self.robot_info = arm_info;
            self.n_states = 2 * self.robot_info.num_q;
            self.position_indices = 1:2:self.n_states;
            self.velocity_indices = 2:2:self.n_states;
            
            % initialize
            self.reset();
        end
        
        function reset(self,state,joint_speeds)
            self.vdisp('Resetting states',LogLevel.INFO) ;
            % reset to zero by default
            self.state = zeros(self.n_states,1) ;
            % Add state
            if nargin > 1
                % changed to be more general
                if length(state) == self.robot_info.num_q
                    % fill in joint positions if they are provided
                    self.vdisp('Using provided joint positions',LogLevel.DEBUG)
                    self.state(self.position_indices) = state ;
                    
                    if nargin > 2
                        % fill in joint speeds if they are provided
                        self.vdisp('Using provided joint speeds',LogLevel.DEBUG)
                        self.state(self.velocity_indices) = joint_speeds ;
                    end
                elseif length(state) == self.n_states
                    % fill in full position and speed state if provided
                    self.vdisp('Using provided full state',LogLevel.DEBUG)
                    self.state = state ;
                else
                    error('Input has incorrect number of states!')
                end
            end
            self.vdisp('Resetting time and inputs',LogLevel.INFO)
            self.time = 0 ;
        end
        
        function state = get_state(self, time)
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
        
        function commit_state_data(self,T_state,Z_state)
            % method: commit_move_data(T_state,Z_state)
            %
            % After moving the agent, commit the new state and input
            % trajectories, and associated time vectors, to the agent's
            % state, time, input, and input_time properties.
            
            % update the time and state
            self.time = [self.time, self.time(end) + T_state(2:end)] ;
            self.state = [self.state, Z_state(:,2:end)] ;
        end
        
        function position = get.position(self)
            position = self.state(self.position_indices,:);
        end
        function velocity = get.velocity(self)
            velocity = self.state(self.velocity_indices,:);
        end
    end
end