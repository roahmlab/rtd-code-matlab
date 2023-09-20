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

        function [box, timed_out] = randomizeBox(world_bounds, size_range, options)
            arguments
                world_bounds(2,3) double
                size_range(1,2) double
                options.creation_buffer(1,1) double = 0
                options.collision_system(:,1) = []
                options.timeout(1,1) double = 10
                options.extra_options(1,1) struct = struct()
            end

            randomizing = true;
            start_tic = tic;
            t_cur = toc(start_tic);
            while randomizing && t_cur <= options.timeout
                % create center, side lengths
                center = ...
                    rtd.random.deprecation.rand_range( world_bounds(1,:) + size_range(2)/2,...
                                     world_bounds(2,:) - size_range(2)/2 );
                side_lengths = ...
                    rtd.random.deprecation.rand_range(size_range(1),...
                                          size_range(2),...
                                          [],[],...
                                          1, 3); % 3 is the dim of the world in this case
                % Create obstacle
                optionsStruct = options.extra_options;
                optionsStruct.component_options.info.creation_buffer = options.creation_buffer;
                prop_box = rtd.entity.BoxObstacle.makeBox(center, side_lengths, options=optionsStruct);

                % test it if we have a collision system
                if ~isempty(options.collision_system)
                    proposal_collision = prop_box.collision.getCollisionObject(buffered=true);
                    [randomizing, ~] = options.collision_system.checkCollisionObject(proposal_collision);
                else
                    randomizing = false;
                end
                t_cur = toc(start_tic);
            end
            box = prop_box;
            timed_out = t_cur > options.timeout;
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