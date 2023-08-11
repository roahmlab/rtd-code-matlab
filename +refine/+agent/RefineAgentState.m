classdef RefineAgentState < rtd.entity.components.BaseStateComponent & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle
    properties
        entity_info = refine.agent.RefineAgentInfo.empty()
        mergeoptions
        n_states = 0
        state double = [] %x,y,h
        time = []

    end

    properties(Dependent)
        position %x,y,h
        velocity %u,v
    end

     methods (Static)
        function options = defaultoptions()
            options.initial_position = [];
            options.initial_velocity = [];
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end

    methods
        function self = RefineAgentState(refine_info, optionsStruct, options)
            arguments
                refine_info refine.agent.RefineAgentInfo
                optionsStruct struct = struct()
                options.initial_position
                options.initial_velocity
                options.verboseLevel
                options.name
                %options.pos_range --> position range for refine?
                %options.vel_range
            end

            options.name = 'highway_cruiser_state';%refine agent state
            self.mergeoptions(optionsStruct, options);

            %setup
            self.entity_info = refine_info;

            %number of states
            self.n_states = refine_info.n_states;

            %reset
            refine_info.reset();

        end

        
        
        % TODO: split into random_state, set_state
        function random_init(self, options)
            arguments
                self
                options.pos_range = []
                options.vel_range = []
                options.random_position = true
                options.random_velocity = false
                options.save_to_options = false
            end
            % Just generate the random configurations --> CHANGE
%             pos_range = options.pos_range;
%             vel_range = options.vel_range;
            
            % reset
            self.state = zeros(self.n_states,1);
            self.time = 0;
            self.step_start_idxs = 0;
            

            
            % Take these initials and merge them in again.
            if options.save_to_options
                merge.initial_position = self.position;
                merge.initial_velocity = self.velocity;
                self.mergeoptions(merge);
            end
        end
        
        function state = get_state(self, time)
            arguments
                self
                time = self.time(end);
            end
            state = refine.entity.states.RobotState(self);%robot state takes in z
            
            % Default to the last time and state
            state.time = time;
            state.state = repmat(self.state(:,end),1,length(time));%state should
            
            % If we can and need to interpolate the state, do it
            mask = time <= self.time(end);
            if length(self.time) > 1 && any(mask)
                state.state(:,mask) = interp1(self.time, self.state.', time(mask)).';
            end
        end

        function commit_state_data(self,T_state,Z_state)
          
            % update the time and state
            self.step_start_idxs = [self.time, length(self.time) + 1]; 
            self.time = [self.time, self.time(end) + T_state(2:end)] ;
            self.state = [self.state, Z_state(:,2:end)] ;%only generators
            
        end

        function position = get.position(self)
            position = self.state(self.entity_info.position_indices,:);
        end
        function velocity = get.velocity(self)
            velocity = self.state(self.entity_info.velocity_indices,:);
        end
    end
end
