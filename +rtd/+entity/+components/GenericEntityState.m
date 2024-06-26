classdef GenericEntityState < rtd.entity.components.BaseStateComponent & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle
    
    % Old functions
    % rand_range
    
    % Inherited properties that must be defined
    properties
        % General information
        entity_info = rtd.entity.components.EmptyEntityInfo.empty()
        
        % state space representation
        n_states = 0
        state double = []
        time = []
    end
    
    methods (Static)
        function options = defaultoptions()
            options.initial_state = [];
            options.n_states = [];
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end
    
    methods
        function self = GenericEntityState(entity_info, optionsStruct, options)
            arguments
                entity_info rtd.entity.components.BaseInfoComponent
                optionsStruct.options struct = struct()
                options.initial_state
                options.n_states
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct.options, options);
            
            % Setup
            self.entity_info = entity_info;
            
            % initialize
            % self.reset();
        end
        
        function reset(self, optionsStruct, options)
            arguments
                self rtd.entity.components.GenericEntityState
                optionsStruct.options struct = struct()
                options.initial_state
                options.n_states
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct.options, options);
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            % Make
            % select the number of states either from the dimension of the
            % entity, or from the option passed in
            if isempty(options.n_states)
                self.n_states = self.entity_info.dimension;
            else
                self.n_states = options.n_states;
            end
            
            % Either start with zeros or use the provided initial state if
            % given
            if isempty(options.initial_state)
                self.state = zeros(self.n_states,1);
            else
                if length(options.initial_state) ~= self.n_states
                    error("Provided initial state has the wrong number of states!")
                end
                self.state = options.initial_state(:);
            end
            
            % Start at time 0
            self.time = 0;
        end
        
        function random_init(self, state_range, save_to_options)
            arguments
                self
                state_range
                save_to_options = false
            end
            % reset
            self.state = zeros(self.n_states,1);
            self.time = 0;
            
            % Make the random configuration
            self.state(:) = rtd.random.deprecation.rand_range(state_range(1,:),state_range(2,:))';
            
            % Take these initials and merge them in again.
            if save_to_options
                merge.initial_state = self.state;
                self.mergeoptions(merge);
            end
        end
        
        function state_out = get_state(self, time)
            arguments
                self rtd.entity.components.GenericEntityState
                time(1,:) double = self.time(end)
            end
            % Default to the last time and state
            state_data = repmat(self.state(:,end),1,length(time));
            
            % If we can and need to interpolate the state, do it
            mask = time <= self.time(end);
            if length(self.time) > 1 && any(mask)
                state_data(:,mask) = interp1(self.time, self.state.', time).';
            end
            
            state_out(length(time)) = rtd.entity.states.GenericEntityStateInstance();
            state_out.setTimes(time);
            state_out.setStateSpace(state_data);
        end
        
        function commit_state_data(self,T_state,Z_state)
            % update the time and state
            self.time = [self.time, self.time(end) + T_state(1:end)] ;
            self.state = [self.state, Z_state(:,1:end)] ;
        end
        
        function set_state(self, state, time)
            arguments
                self rtd.entity.components.GenericEntityState
                state(:,1) double
                time(1,:) double = self.time(end)
            end
            if length(state) ~= self.n_states
                error("Dimension of state provided doesn't match n_states!")
            end
            % replace the state
            self.time = time;
            self.state = state;
        end
    end
end
