classdef StateComponent < handle
    %STATECOMPONENT Data and Behavior for generic states
    % TODO migrate out
    
    properties
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
        function self = StateComponent(dimension)
            self.n_states = 2 * dimension;
            self.position_indices = 1:2:self.n_states;
            self.velocity_indices = 2:2:self.n_states;
            
            % initialize
            self.reset();
        end
        
        function reset(self,state_or_position,velocity)
            % TODO Add back vdisp
            %self.vdisp('Resetting states',3) ;
            
            % reset to zero by default
            self.state = zeros(self.n_states,1) ;
            % TODO Move to controller
            %self.reference_state = zeros(self.n_states,1) ;
            %self.reference_acceleration = zeros(self.n_states/2,1) ;
            
            if nargin > 1
                % changed to be more general
                if length(state_or_position) == self.n_states / 2
                    % fill in joint positions if they are provided
                    %self.vdisp('Using provided joint positions',6)
                    self.state(self.position_indices) = state_or_position ;
                    %self.reference_state(self.position_indices) = state ;
                    
                    if nargin > 2
                        % fill in joint speeds if they are provided
                        %self.vdisp('Using provided joint speeds',6)
                        self.state(self.velocity_indices) = velocity ;
                        %self.reference_state(self.velocity_indices) = joint_speeds ;
                    end
                elseif length(state_or_position) == self.n_states
                    % fill in full position and speed state if provided
                    %self.vdisp('Using provided full state',6)
                    self.state = state_or_position ;
                    %self.reference_state = state ;
                else
                    error('Input has incorrect number of states!')
                end
            end
            
            %self.vdisp('Resetting time and inputs',3)
            self.time = 0 ;
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
        end
    end    
end

