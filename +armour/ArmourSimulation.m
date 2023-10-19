classdef ArmourSimulation < rtd.sim.BaseSimulation & handle
    % Inherited properties we want to define
    properties
        % We plan and run ARMOUR at half second intervals
        simulation_timestep = 0.5
    end

    % These are extra references to entities and systems stored in the world model just for convenience
    properties
        % Reference to the primary arm agent entity for convenience
        agent

        % Reference to the collision system for convenience
        collision_system

        % Reference to the visual system for convenience
        visual_system

        % Reference to the goal system for convenience
        goal_system
    end
    
    % Simulation Methods
    methods
        % Important stuff to get started
        % TODO breakout options
        function self = ArmourSimulation(optionsStruct, options)
            arguments
                optionsStruct.options struct = struct()
                options.simulation_timestep
            end
            self.simulation_state = 'CONSTRUCTED';
        end
       
        function setup(self, agent)
            % Setup the entity and systems for the simulation
            %
            % Resets the simulation to a blank state and sets up the
            % simulation with the given ArmourAgent. For this simulation,
            % this must be called before initialize or import as the
            % ArmourAgent class is currently not fully serializable.
            %
            % Arguments:
            %   agent: The ArmourAgent to use for the simulation
            %
            arguments
                self(1,1) armour.ArmourSimulation
                agent(1,1) armour.ArmourAgent
            end

            self.simulation_state = 'SETTING_UP';

            self.world = rtd.sim.world.WorldModel;
            
            self.agent = agent;
            self.world.addEntity(agent, type='dynamic');

            % Initialize the visual and collision systems
            self.visual_system = rtd.sim.systems.patch_visual.PatchVisualSystem;
            self.collision_system = rtd.sim.systems.patch3d_collision.Patch3dCollisionSystem(time_discretization=0.01);
            self.world.addSystem(self.visual_system, 'visual_system');
            self.world.addSystem(self.collision_system, 'collision_system');

            self.collision_system.addObjects(dynamic=self.agent.collision)
            self.visual_system.addObjects(dynamic=self.agent.visual)

            self.goal_system = armour.deprecation.RandomArmConfigurationGoal(self.collision_system, self.agent);
            self.world.addSystem(self.goal_system, 'goal_system');
            
            % reset the log
            % For other simulations, you might want to validate keys and
            % manually add them to ensure the summary section works as
            % expected.
            self.simulation_log = rtd.util.containers.VarLogger(validate_keys=false);
            self.simulation_time = 0;
            self.simulation_state = 'SETUP_READY';
        end
        
        function import(self, filename)
            % Import all configurations for the simulation world from an XML file
            %
            % This function will import all entities and systems from an
            % XML file. This needs to be called after setup and will
            % overwrite any existing entities and systems.
            %
            % Arguments:
            %   filename: The filename of the XML file to import
            %
            arguments
                self(1,1) armour.ArmourSimulation
                filename {mustBeFile}
            end

            if self.simulation_state > "INITIALIZING"
                self.visual_system.close();
                self.setup(self.agent);
            end
            self.simulation_state = 'INITIALIZING';
            self.world.mergeFromXML(filename)
            
            % Add collision objects to the collision system
            static_collisions = self.world.getEntityComponent('collision', type='static');
            static_collision_objects = cellfun(@(x)(static_collisions.(x).getCollisionObject()),fieldnames(static_collisions));

            dynamic_collisions = self.world.getEntityComponent('collision', type='dynamic');
            dynamic_collision_objects = cellfun(@(x)(dynamic_collisions.(x)),fieldnames(dynamic_collisions));

            self.collision_system.addObjects(static=static_collision_objects, dynamic=dynamic_collision_objects);

            % add visual objects to the visual system
            static_visuals = self.world.getEntityComponent('visual', type='static');
            static_visual_objects = cellfun(@(x)(static_visuals.(x)),fieldnames(static_visuals));

            dynamic_visuals = self.world.getEntityComponent('visual', type='dynamic');
            dynamic_visual_objects = cellfun(@(x)(dynamic_visuals.(x)),fieldnames(dynamic_visuals));

            self.visual_system.addObjects(static=static_visual_objects, dynamic=dynamic_visual_objects);

            % Add the collision system
            self.visual_system.addObjects(static=self.goal_system);

            % Reset the visuals and ready the sim
            self.visual_system.redraw();
            self.simulation_state = 'READY';
        end

        function initialize(self)
            % Initialize the simulation
            %
            % This function will initialize the simulation. This needs to
            % be called after setup and will ready the simulation for
            % stepping.
            %
            arguments
                self(1,1) armour.ArmourSimulation
            end

            if self.simulation_state > "INITIALIZING"
                error("This simulation currently does not support reinitialization without resetup");
            end
            self.simulation_state = 'INITIALIZING';

            % A lot of temporaries
            timeout = 10;
            % Create a random start (assuming no obstacles)
            randomizing = true;
            start_tic = tic;
            t_cur = toc(start_tic);
            while randomizing && t_cur <= timeout
                self.agent.state.random_init(save_to_options=true);
                proposal_obj = self.agent.collision.getCollisionObject();

                % test it in the collision system
                [randomizing, ~] = self.collision_system.checkCollisionObject(proposal_obj);
                t_cur = toc(start_tic);
            end

            % Create the random obstacles
            n_obstacles = 3;
            obstacle_size_range = [0.01 0.5] ; % [min, max] side length
            creation_buffer = 0.05;
            world_bounds = [self.agent.info.reach_limits(1:2:6); self.agent.info.reach_limits(2:2:6)];
            for i=1:n_obstacles
                % Randomize an obstacle
                [box, timed_out] = rtd.entity.BoxObstacle.randomizeBox(world_bounds, ...
                    obstacle_size_range, creation_buffer=creation_buffer, ...
                    collision_system=self.collision_system, timeout=timeout);
                if timed_out
                    warning("Timeout occured when creating obstacle, obstacle may be overlapping with robot configuration!")
                end

                % Add it
                self.world.addEntity(box);
                self.collision_system.addObjects(static=box.collision.getCollisionObject)
                self.visual_system.addObjects(static=box.visual)
            end

            % Create the goal
            self.goal_system.reset();
            self.goal_system.createGoal();
            % Save the goal position (workaround for the deprecated goal
            % system)
            self.goal_system.reset(goal_position=self.goal_system.goal_position);
            self.visual_system.addObjects(static=self.goal_system);

            % reset the agent (We saved the position to options earlier)
            self.agent.reset

            % redraw
            self.visual_system.redraw();
            
            self.simulation_state = 'READY';
        end
    end

    % Protected methods for stepping
    methods(Access=protected)
        % Pre-step doesn't exist for this simulation

        % Step the simulation
        function info = step_impl(self)
            % Update entities
            agent_results = self.agent.update(self.simulation_timestep);
            
            % Update systems
            [collision, contactPairs] = self.collision_system.updateCollision(self.simulation_timestep);

            if collision
                disp("Collision Detected, Breakpoint!")
                keyboard
                disp("Continuing")
            end
            info.agent_results = agent_results;
            info.collision = collision;
            info.contactPairs = contactPairs;
        end
        
        % Post-step goal checks and visual updates
        function info = post_step_impl(self)
            % Check if goal was achieved
            goal = self.goal_system.updateGoal(self.simulation_timestep);
            pause_requested = self.visual_system.updateVisual(self.simulation_timestep);
            info.goal = goal;
            info.pause_requested = pause_requested;
        end
    end
end
