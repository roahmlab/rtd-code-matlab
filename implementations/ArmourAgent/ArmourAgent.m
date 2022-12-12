classdef ArmourAgent < OptionsClass & NamedClass & handle
% ArmourAgent
% The Agent with the robust controller for ARMOUR
% Left to do are the helper safety check functions like check input limits
% and glueing this together
    properties
        %%%%%%%%%%%%%%%%%%%
        % Data Components %
        %%%%%%%%%%%%%%%%%%%
        
        % Core data to describe the agent.
        % Should be mostly invariant.
        info ArmourAgentInfo
        
        % The changing values and their history that fully describe the
        % state of the agent at any given (valid) point in time
        state
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Prerequisite Utility Components %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % How do we calculate the forward and inverse kinematics for the
        % agent?
        kinematics
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Behavior + Data Components %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % What is the control input(s) to this agent for a chosen
        % trajectory or similar?
        % This also stores relevant controller info and history of
        % trajectories if applicable.
        controller
        
        % What is the evolution of the agent's state given some control
        % input?
        % This stores the history of the control inputs used for the
        % dynamics.
        dynamics
        
        
        %%%%%%%%%%%%%%%%%%%%%%%
        % Behavior Components %
        %%%%%%%%%%%%%%%%%%%%%%%
        
        % What is the geometry that is actually relevant to the collision
        % system or is the agent in collision?
        collision
        
        % What is the resulting visual geometry or rendered result for
        % whatever visualization system we are using?
        visual
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Other Utility Components %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % None
        
    end
    properties (Dependent)
        uuid
    end
    
    methods (Static)
        function options = defaultoptions()
            components.info = 'ArmourAgentInfo';
            components.state = 'ArmourAgentState';
            components.kinematics = 'ArmKinematics';
            components.controller = 'ArmourController';
            components.dynamics = 'ArmourDynamics';
            components.collision = 'ArmourPatchCollision';
            components.visual = 'ArmourPatchVisual';
            options.components = components;
            options.component_options = struct;%get_componentoptions(components);
            options.verboseLevel = LogLevel.INFO;
            options.name = '';
        end
        
        function self = from_options(robot, params, options)
            a = ArmourAgentInfo(robot, params, options.component_options.info);
            self = ArmourAgent(a, [], [], [], [], [], [], options);
        end
    end
    
    methods
        function self = ArmourAgent(info, components, optionsStruct, options)
            arguments
                info
                components.state_component = []
                components.kinematics_component = []
                components.controller_component = []
                components.dynamics_component = []
                components.collision_component = []
                components.visual_component = []
                optionsStruct.optionsStruct struct = struct()
                options.components
                options.component_options
                options.verboseLevel
                options.name
            end
            % mark each of our preconstructed components and save them
            names = {'info', 'state', 'kinematics', 'controller', 'dynamics', 'collision', 'visual'};
            components = {info, components.state_component, ...
                components.kinematics_component, components.controller_component, ...
                components.dynamics_component, components.collision_component, ...
                components.visual_component};
            
            % Boolean mask for provided components
            provided_components_mask = ~cellfun(@isempty, components);
            provided_components = components(provided_components_mask);
            
            % Preprocess and save the names
            provided_component_classnames = cellfun(@class, provided_components, 'UniformOutput', false);
            provided_component_names = names(provided_components_mask);
            override_options.components = [provided_component_names; provided_component_classnames];
            override_options.components = struct(override_options.components{:});
            
            % Preprocess the instances for options
            component_handles = [provided_component_names; provided_components];
            override_options.component_options = get_componentoptions(struct(component_handles{:}));
            
            % Build option structs for the preconstructed components and
            % save them to this agent.
            for i=1:length(provided_component_names)
                self.(provided_component_names{i}) = provided_components{i};
            end
            
            % Merge all options
            options = self.mergeoptions(optionsStruct.optionsStruct, options, override_options);
            
            % (Re)construct all components for consistency
            self.state = construct_component(options, 'state', self.info);
            self.kinematics = construct_component(options, 'kinematics', self.info, self.state);
            self.controller = construct_component(options, 'controller', self.info, self.state);
            self.dynamics = construct_component(options, 'dynamics', self.info, self.state, self.controller);
            self.collision = construct_component(options, 'collision', self.info, self.state, self.kinematics);
            self.visual = construct_component(options, 'visual', self.info, self.state, self.kinematics);
            
            % Reset
            self.reset()
        end
        
        % Reset all components.
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.component_options
                options.verboseLevel
                options.name
            end
            % Perform an internal update, then merge in options
            self.getoptions();
            options = self.mergeoptions(optionsStruct, options);
            
            % Iterate through all we have options for
            fields = fieldnames(options.component_options).';
            for fieldname = fields
                % Overwrite their names if we have a name for our class
                if ~isempty(options.name) ...
                        && isfield(options.component_options.(fieldname{1}), 'name') ...
                        && isempty(options.component_options.(fieldname{1}).name)
                    options.component_options.(fieldname{1}).name = options.name;
                    
                end
                self.(fieldname{1}).reset(options.component_options.(fieldname{1}));
            end
            
            % Make sure to reset those that don't have options but still
            % need reset
            names = {'info', 'state', 'kinematics', 'controller', 'dynamics', 'collision', 'visual'};
            remaining = setdiff(names, fields);
            for name = remaining
                self.(name{1}).reset();
            end
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
        end
        
        % Get all the options used for initialization of the robot
        function options = getoptions(self)
            % Update all the component options before returning.
            components.info = self.info;
            components.state = self.state;
            components.kinematics = self.kinematics;
            components.controller = self.controller;
            components.dynamics = self.dynamics;
            components.collision = self.collision;
            components.visual = self.visual;
            options.component_options = get_componentoptions(components);
            options = self.mergeoptions(options);
        end
%         
%         % Pass the trajectory to the controller
%         % TODO Decide on relevance
%         function setTrajectory(self, trajectory)
%             self.controller.setTrajectory(trajectory);
%         end
        
        % Lifecycle
        function check = pre_checks(self)
            % This would run before the update in each world step.
        end
        
        % Update this agent's state by t_move
        function update(self, t_move)
            self.dynamics.move(t_move);
        end
        
        % Safety checks
        function check = post_checks(self)
            % TODO make this addable, and check across? Maybe not (reasons
            % not are obfuscation)
            % true means an issue happened!
            check.joint_limits = self.state.joint_limit_check(t_check_step);
            check.control_inputs = self.dynamics.controller_input_check(t_check_step);
            check.ultimate_bound = self.controller.ultimate_bound_check(t_check_step, self.dynamics.controller_log);
        end
        
        function uuid = get.uuid(self)
            uuid = self.info.uuid;
        end
            
    end
end

function component_options = get_componentoptions(components)
    % Generate an entry for each field which is of type OptionsClass
    fields = fieldnames(components).';
    for fieldname = fields
        if ismember('OptionsClass', superclasses(components.(fieldname{1})))
            try
                component_options.(fieldname{1}) ...
                    = components.(fieldname{1}).getoptions();
            catch
                component_options.(fieldname{1}) ...
                    = eval([components.(fieldname{1}) '.defaultoptions()']);
            end
        end
    end
end

function component = construct_component(option_struct, name, varargin)
    if isfield(option_struct, 'component_options') ...
            && isfield(option_struct.component_options, name)
        component = feval(option_struct.components.(name), varargin{:}, option_struct.component_options.(name));
    else
        component = feval(option_struct.components.(name), varargin{:});
    end
end
