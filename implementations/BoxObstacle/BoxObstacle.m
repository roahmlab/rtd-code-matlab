classdef BoxObstacle < OptionsClass & NamedClass & handle
% BoxObstacle
% The Agent with the robust controller for ARMOUR
% Left to do are the helper safety check functions like check input limits
% and glueing this together
    properties
        %%%%%%%%%%%%%%%%%%%
        % Data Components %
        %%%%%%%%%%%%%%%%%%%
        
        % Core data to describe the agent.
        % Should be mostly invariant.
        info
        
        % The changing values and their history that fully describe the
        % state of the agent at any given (valid) point in time
        state
        
        
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
        
        % Generate our desired representation for this obstacle
        representation
        
    end
    properties (Dependent)
        uuid
    end
    
    methods (Static)
        function options = defaultoptions()
            components.info = 'BoxObstacleInfo';
            components.state = 'GenericStateComponent';
            components.collision = 'BoxPatchCollision';
            components.visual = 'BoxPatchVisual';
            components.representation = 'BoxObstacleZonotope';
            options.components = components;
            options.component_options = struct;%get_componentoptions(components);
            options.component_logLevelOverride = [];
            options.verboseLevel = LogLevel.INFO;
            options.name = '';
        end
        
        function box = makeBox(center, side_lengths, optionsStruct)
            arguments
                center
                side_lengths
                optionsStruct struct = struct()
            end
            component_options.info.side_lengths = side_lengths;
            component_options.state.initial_state = center;
            box = BoxObstacle(optionsStruct=optionsStruct, ...
                              component_options=component_options);
        end
    end
    
    methods
        % WIP
        function self = BoxObstacle(components, optionsStruct, options)
            arguments
                components.info_component = []
                components.state_component = []
                components.collision_component = []
                components.visual_component = []
                components.representation_component = []
                optionsStruct.optionsStruct struct = struct()
                options.components
                options.component_options
                options.component_logLevelOverride
                options.verboseLevel
                options.name
            end
            % mark each of our preconstructed components and save them
            names = {'info', 'state', 'collision', 'visual', 'representation'};
            components = {components.info_component, components.state_component, ...
                components.collision_component, components.visual_component, ...
                components.representation_component};
            
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
            self.info = construct_component(options, 'info');
            self.state = construct_component(options, 'state', self.info);
            self.collision = construct_component(options, 'collision', self.info, self.state);
            self.visual = construct_component(options, 'visual', self.info, self.state);
            self.representation = construct_component(options, 'representation', self.info, self.state);
            
            % Reset
            self.reset()
        end
        
        % Reset all components.
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.component_options
                options.component_logLevelOverride
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
                % Overwrite the verboseLevel if we have a logleveloverride
                if ~isempty(options.component_logLevelOverride) ...
                        && isfield(options.component_options.(fieldname{1}), 'verboseLog')
                    options.component_options.(fieldname{1}).verboseLog = options.component_logLevelOverride;
                    
                end
                self.(fieldname{1}).reset(options.component_options.(fieldname{1}));
            end
            
            % Make sure to reset those that don't have options but still
            % need reset
            names = {'info', 'state', 'collision', 'visual', 'representation'};
            remaining = setdiff(names, fields);
            for name = remaining
                self.(name{1}).reset();
            end
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
        end
        
        % Get all the options used for this obstacle
        function options = getoptions(self)
            % Update all the component options before returning.
            components.info = self.info;
            components.state = self.state;
            components.collision = self.collision;
            components.visual = self.visual;
            components.representation = self.representation;
            options.component_options = get_componentoptions(components);
            options = self.mergeoptions(options);
        end
        
        % Lifecycle
        function check = pre_checks(self)
            % This would run before the update in each world step.
        end
        
        % Update this entity's state by t_move
        function update(self, t_move)
            % NOP
        end
        
        % Safety checks
        function check = post_checks(self)
            % TODO make this addable, and check across? Maybe not (reasons
            % not are obfuscation)
            % true means an issue happened!
        end
        
        function uuid = get.uuid(self)
            uuid = self.info.uuid;
        end
    end
end

function component_options = get_componentoptions(components)
    % Generate an entry for each field which is of type OptionsClass
    fields = fieldnames(components).';
    component_options = struct;
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