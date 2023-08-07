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

            %reset
            self.reset();

        end

        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.initial_position
                options.initial_velocity
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct, options);
            
            % Set component dependent properties
            self.n_states = 10; %from highway_cruising_10_state_agent
            self.position_indices = 1:2:self.n_states;%x,y,h
            self.velocity_indices = 2:2:self.n_states;%u,v
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            % reset the rest
            self.vdisp('Resetting time and states', 'INFO');
            
            % reset to zero by default
            self.state = zeros(self.n_states,1);
            self.time = 0;
            self.step_start_idxs = 0;
            
            % Add position
            if ~isempty(options.initial_position)
                self.vdisp('Using provided positions', 'TRACE') %x,y,h
                self.state(self.position_indices) = options.initial_position(:);
            end
            
            % Add velocity
            if ~isempty(options.initial_velocity)
                self.vdisp('Using provided longitudnal and lateral velocities', 'TRACE')%u0,v0
                self.state(self.velocity_indices) = options.initial_velocity(:);
            end
            
            % Take these initials and merge them in again.
            options.initial_position = self.position;
            options.initial_velocity = self.velocity;
            self.mergeoptions(options);
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
            pos_range = options.pos_range;
            vel_range = options.vel_range;
            if isempty(pos_range)
                pos_range = [self.entity_info.joints(1:self.entity_info.num_q).position_limits];%check the para of entity_info
            end
            if isempty(vel_range)
                vel_range = [self.entity_info.joints(1:self.entity_info.num_q).velocity_limits];
            end
            
            % set any joint limits that are +Inf to pi and -Inf to -pi
            pos_range_infs = isinf(pos_range) ;
            pos_range(1,pos_range_infs(1,:)) = -pi ;
            pos_range(2,pos_range_infs(2,:)) = +pi ;
            
            % reset
            self.state = zeros(self.n_states,1);
            self.time = 0;
            self.step_start_idxs = 0;
            
            % Make the random configuration
            if options.random_position
                self.state(self.position_indices) = ...
                    rtd.random.deprecation.rand_range(pos_range(1,:),pos_range(2,:))';
            end
            if options.random_velocity
                self.state(self.velocity_indices) = ...
                    rtd.random.deprecation.rand_range(vel_range(1,:),vel_range(2,:))';
            end
            
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
            state = refine.entity.states.RobotState(self.position_indices, self.velocity_indices);%robot state takes in z
            
            % Default to the last time and state
            state.time = time;
            state.state = repmat(self.state(:,end),1,length(time));
            
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