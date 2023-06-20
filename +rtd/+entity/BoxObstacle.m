classdef BoxObstacle < rtd.sim.world.WorldEntity & handle
% BoxObstacle
% The Agent with the robust controller for ARMOUR
% Left to do are the helper safety check functions like check input limits
% and glueing this together
    % Required Abstract Properties
    properties
        %%%%%%%%%%%%%%%%%%%
        % Data Components %
        %%%%%%%%%%%%%%%%%%%
        
        % Core data to describe the agent.
        % Should be mostly invariant.
        info = rtd.entity.box_obstacle.BoxObstacleInfo.empty()
        
        % The changing values and their history that fully describe the
        % state of the agent at any given (valid) point in time
        state = rtd.entity.components.GenericEntityState.empty()
    end

    % Specific for this entity
    properties     
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
    
    methods (Static)
        function options = defaultoptions()
            options = rtd.sim.world.WorldEntity.baseoptions();
            
            % These are the names for the default components
            components.info = 'rtd.entity.box_obstacle.BoxObstacleInfo';
            components.state = 'rtd.entity.components.GenericEntityState';
            components.collision = 'rtd.entity.box_obstacle.BoxPatchCollision';
            components.visual = 'rtd.entity.box_obstacle.BoxPatchVisual';
            components.representation = 'rtd.entity.box_obstacle.BoxObstacleZonotope';
            options.components = components;
        end
        
        function box = makeBox(center, side_lengths, optionsStruct)
            arguments
                center
                side_lengths
                optionsStruct.options struct = struct()
            end
            component_options.info.side_lengths = side_lengths;
            component_options.state.initial_state = center;
            box = rtd.entity.BoxObstacle(options=optionsStruct.options, ...
                              component_options=component_options);
        end
    end
    
    methods
        % WIP
        function self = BoxObstacle(components, optionsStruct, options)
            arguments
                components.info = []
                components.state = []
                components.collision = []
                components.visual = []
                components.representation = []
                optionsStruct.options struct = struct()
                options.components
                options.component_options
                options.component_logLevelOverride
                options.verboseLevel
                options.name
            end
            % Get override options based on provided components
            override_options = rtd.sim.world.WorldEntity.get_componentOverrideOptions(components);

            % Merge all options
            self.mergeoptions(optionsStruct.options, options, override_options);
            
            % (Re)construct all components for consistency
            self.construct_component('info');
            self.construct_component('state', self.info);
            self.construct_component('collision', self.info, self.state);
            self.construct_component('visual', self.info, self.state);
            self.construct_component('representation', self.info, self.state);
            
            % Reset
            self.reset()
        end
        
        % Reset all components.
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct.options struct = struct()
                options.component_options
                options.component_logLevelOverride
                options.verboseLevel
                options.name
            end
            % Perform an internal update, then merge in options
            self.getoptions();
            options = self.mergeoptions(optionsStruct.options, options);
                        
            % reset all components
            self.reset_components()
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
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
    end
end