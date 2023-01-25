classdef ArmourAgentState < rtd.entity.components.BaseStateComponent & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle
    
    % Old functions
    % rand_range
    % Inherited properties that must be defined
    properties
        % General information of the robot arm
        entity_info = ArmourAgentInfo.empty()
        
        % state space representation
        n_states = 0
        state double = []
        time = []
    end
    
    % Extra properties we define
    properties
        % Indexes for our checking of each step.
        step_start_idxs = []
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
            options.initial_position = [];
            options.initial_velocity = [];
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end
    
    methods
        function self = ArmourAgentState(arm_info, optionsStruct, options)
            arguments
                arm_info ArmourAgentInfo
                optionsStruct struct = struct()
                options.initial_position
                options.initial_velocity
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct, options);
            
            % Setup
            self.entity_info = arm_info;
            
            % initialize
            % self.reset();
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
            self.n_states = 2 * self.entity_info.num_q;
            self.position_indices = 1:2:self.n_states;
            self.velocity_indices = 2:2:self.n_states;
            
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
                self.vdisp('Using provided joint positions', 'TRACE')
                self.state(self.position_indices) = options.initial_position(:);
            end
            
            % Add velocity
            if ~isempty(options.initial_velocity)
                self.vdisp('Using provided joint velocities', 'TRACE')
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
            % Just generate the random configurations
            pos_range = options.pos_range;
            vel_range = options.vel_range;
            if isempty(pos_range)
                pos_range = [self.entity_info.joints(1:self.entity_info.num_q).position_limits];
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
                self.state(self.position_indices) = rand_range(pos_range(1,:),pos_range(2,:))';
            end
            if options.random_velocity
                self.state(self.velocity_indices) = rand_range(vel_range(1,:),vel_range(2,:))';
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
            state = ArmRobotState(self.position_indices, self.velocity_indices);
            
            % Default to the last time and state
            state.time = time;
            state.state = repmat(self.state(:,end),1,length(time));
            
            % If we can and need to interpolate the state, do it
            mask = time <= self.time(end);
            if length(self.time) > 1 && any(mask)
                state.state(:,mask) = interp1(self.time, self.state.', time(mask)).';
            end
        end
        
        % function state = get_step_states(self, step)
        %     arguments
        %         self
        %         step = length(self.step_start_idxs)
        %     end
        %     start_idx = self.step_start_idxs(step);
        %     end_idx = length(self.time);
            
        %     if step ~= length(self.step_start_idxs)
        %         end_idx = self.step_start_idxs(step+1);
        %     end
            
        %     % TODO hash out
        % end
        
        function commit_state_data(self,T_state,Z_state)
            % method: commit_move_data(T_state,Z_state)
            %
            % After moving the agent, commit the new state and input
            % trajectories, and associated time vectors, to the agent's
            % state, time, input, and input_time properties.
            
            % update the time and state
            self.step_start_idxs = [self.time, length(self.time) + 1]; 
            self.time = [self.time, self.time(end) + T_state(2:end)] ;
            self.state = [self.state, Z_state(:,2:end)] ;
            % TODO, remove magic number 2
        end

        function out = joint_limit_check(self, t_check_step)
            % create time vector for checking
            start_idx = self.step_start_idxs(end);
            t_check = self.time(start_idx):t_check_step:self.time(end);

            % get agent state trajectories interpolated to time
            pos_check = interp1(self.time(start_idx:end), self.position(:,start_idx:end).', t_check).';
            vel_check = interp1(self.time(start_idx:end), self.velocity(:,start_idx:end).', t_check).';

            % check bound satisfaction
            self.vdisp('Running joint limits check!', 'INFO');
            
            % check position & velocity
            pos_exceeded = false(size(pos_check));
            vel_exceeded = false(size(vel_check));
            for idx=1:self.entity_info.num_q
                % pos
                lb = pos_check(idx,:) < self.entity_info.joints(idx).position_limits(1);
                ub = pos_check(idx,:) > self.entity_info.joints(idx).position_limits(2);
                pos_exceeded(idx,:) = lb + ub;
                % vel
                lb = vel_check(idx,:) < self.entity_info.joints(idx).velocity_limits(1);
                ub = vel_check(idx,:) > self.entity_info.joints(idx).velocity_limits(2);
                vel_exceeded(idx,:) = lb + ub;
            end
            
            % Get out results
            out = any(pos_exceeded, 'all') || any(vel_exceeded, 'all');
            if out
                % Position limit exceeded in these positions
                [joint_idx_list, t_idx_list] = find(pos_exceeded);
                for idx = 1:length(joint_idx_list)
                    t_idx = t_idx_list(idx);
                    joint_idx = joint_idx_list(idx);
                    % Format error message
                    msg = sprintf('t=%.2f, %d-position limit exceeded: %.5f vs [%.5f, %.5f]',...
                        t_check(t_idx), joint_idx, pos_check(joint_idx, t_idx), ...
                        self.entity_info.joints(joint_idx).position_limits(1), ...
                        self.entity_info.joints(joint_idx).position_limits(2));
                    self.vdisp(msg, 'ERROR');
                end
                
                % Velocity limit exceeded in these positions
                [joint_idx_list, t_idx_list] = find(vel_exceeded);
                for idx = 1:length(joint_idx_list)
                    t_idx = t_idx_list(idx);
                    joint_idx = joint_idx_list(idx);
                    % Format error message
                    msg = sprintf('t=%.2f, %d-velocity limit exceeded: %.5f vs [%.5f, %.5f]',...
                        t_check(t_idx), joint_idx, vel_check(joint_idx, t_idx), ...
                        self.entity_info.joints(joint_idx).velocity_limits(1), ...
                        self.entity_info.joints(joint_idx).velocity_limits(2));
                    self.vdisp(msg, 'ERROR');
                end
            else
                self.vdisp('No joint limits exceeded', 'INFO');
            end
        end
        
        function position = get.position(self)
            position = self.state(self.position_indices,:);
        end
        function velocity = get.velocity(self)
            velocity = self.state(self.velocity_indices,:);
        end
    end
end