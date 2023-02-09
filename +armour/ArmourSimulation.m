classdef ArmourSimulation < rtd.sim.BaseSimulation & handle
    properties
        simulation_timestep = 0.5
        world = struct
        world_by_uuid = struct
        entities
        %systems
        simulation_log rtd.util.containers.VarLogger = rtd.util.containers.VarLogger.empty()
    end
    properties
        agent
        obstacles
        collision_system
        visual_system
        goal_system
    end
    events
        PreStep
        Step
        PostStep
        NewObjectAdded
    end
    methods
        % Important stuff to get started
        function self = ArmourSimulation(optionsStruct, options)
            arguments
                optionsStruct struct = struct()
                options.simulation_timestep
            end
            self.simulation_state = 'CONSTRUCTED';
        end
        
        % Add object
        function add_object(self, object, options)
            arguments
                self armour.ArmourSimulation
                object
                options.isentity = false
                options.update_name = false
                options.collision = []
                options.visual = []
            end
            
            % Add the object to the world
            % Create a name for the object based on its classname if it
            % doesn't have a given name.
            name = object.name;
            if isempty(name)
                % Base classname
                name = object.classname;
                % Get number to append by summing a regexp. It returns the
                % index of the occurance in each string, but since it's
                % limited to the first word anyway, it'll be 1 or empty.
                search_string = ['^', char(name), '\d+$'];
                all_names = fieldnames(self.world);
                id = sum(cell2mat(regexp(all_names, search_string)));
                name = [char(name), num2str(id)];
            end
            if options.update_name
                object.update_name(name);
            end
            self.world.(name) = object;
            
            % Add to the entity list if it's an entity
            if options.isentity
                self.entities = [self.entities, {object}];
                % Add the collision component provided to the collision system
                if ~isempty(options.collision)
                    self.collision_system.addObjects(dynamic=options.collision);
                end
                % Add the visualization component provided to the visual
                % system
                if ~isempty(options.visual)
                    self.visual_system.addObjects(dynamic=options.visual);
                end
            % if it's not, check for and add to collision or visual
            else
                if ~isempty(options.collision)
                    self.collision_system.addObjects(static=options.collision);
                end
                % Add the visualization component provided to the visual
                % system
                if ~isempty(options.visual)
                    self.visual_system.addObjects(static=options.visual);
                end
            end

            % TODO setup custom event data to return the object added
            notify(self, 'NewObjectAdded')
        end
        
        function setup(self, agent)
            arguments
                self armour.ArmourSimulation
                agent armour.ArmourAgent
            end
            if self.simulation_state > "SETTING_UP"
                self.world = struct;
                self.entities = [];
            end
            self.simulation_state = 'SETTING_UP';
            
            self.agent = agent;
            % Initialize the visual and collision systems
            self.visual_system = rtd.sim.systems.patch_visual.PatchVisualSystem;
            self.collision_system = rtd.sim.systems.patch3d_collision.Patch3dCollisionSystem(time_discretization=0.01);
            %self.systems = {self.visual_system, self.collision_system};
            
            % add the agent
            self.add_object(agent, isentity=true, collision=agent.collision, visual=agent.visual);

%             % Create the base obstacles
%             base_creation_buffer = 0.025;
%             face_color = [0.5 0.5 0.5];
%             edge_color = [0 0 0];
% 
%             base_options.info.is_base_obstacle = true;
%             base_options.info.creation_buffer = base_creation_buffer;
%             base_options.visual.face_color = face_color;
%             base_options.visual.edge_color = edge_color;
%             optionsStruct.component_options = base_options;
%             base = rtd.entity.BoxObstacle.makeBox( [-0.0580; 0; 0.1778], ...
%                                         2*[0.2794, 0.2794, 0.1778], ...
%                                         optionsStruct);
%             tower = rtd.entity.BoxObstacle.makeBox([-0.2359; 0; 0.6868], ...
%                                         2*[0.1016, 0.1651, 0.3312], ...
%                                         optionsStruct);
%             head = rtd.entity.BoxObstacle.makeBox( [-0.0580; 0; 1.0816], ...
%                                         2*[0.1651, 0.1397, 0.0635], ...
%                                         optionsStruct);
%             % Floor
%             floor_color = [0.9, 0.9, 0.9];
%             optionsStruct.component_options.visual.face_color = floor_color;
%             floor = rtd.entity.BoxObstacle.makeBox([-0.0331;0;0.005], ...
%                                         2*[1.3598, 1.3598, 0.0025], ...
%                                         optionsStruct);
% 
%             % Add them to the world
%             for obs = [base, tower, head, floor]
%                 self.add_object(obs, collision=obs.collision.getCollisionObject, visual=obs.visual);
%             end
            
            % reset the log
            % For other simulations, you might want to validate keys and
            % manually add them to ensure the summary section works as
            % expected.
            self.simulation_log = rtd.util.containers.VarLogger(validate_keys=false);

            self.simulation_state = 'SETUP_READY';
        end
        
        function initialize(self)
            if self.simulation_state > "INITIALIZING"
                %error("This simulation currently does not support reinitialization without resetup");
            end
            self.simulation_state = 'INITIALIZING';

            % A lot of temporaries
            timeout = 10;
            % Create a random start (assuming no obstacles)
            randomizing = true;
            start_tic = tic;
            t_cur = toc(start_tic);
            while randomizing && t_cur <= timeout
                self.agent.state.random_init();
                proposal_obj = self.agent.collision.getCollisionObject();

                % test it in the collision system
                [randomizing, pairs] = self.collision_system.checkCollisionObject(proposal_obj);
                t_cur = toc(start_tic);
            end
            % This is captured by the goal generator if we don't set anything as the
            % start.

            % Create the random obstacles
            n_obstacles = 0;
            obstacle_size_range = [0.01 0.5] ; % [min, max] side length
            creation_buffer = 0.05;
            world_bounds = [self.agent.info.reach_limits(1:2:6); self.agent.info.reach_limits(2:2:6)];
            for obs_num = 1:n_obstacles
                randomizing = true;
                start_tic = tic;
                t_cur = toc(start_tic);
                while randomizing && t_cur <= timeout
                    % create center, side lengths
                    center = ...
                        rtd.random.deprecation.rand_range( world_bounds(1,:) + obstacle_size_range(2)/2,...
                                         world_bounds(2,:) - obstacle_size_range(2)/2 );
                    side_lengths = ...
                        rtd.random.deprecation.rand_range(obstacle_size_range(1),...
                                              obstacle_size_range(2),...
                                              [],[],...
                                              1, 3); % 3 is the dim of the world in this case
                    % Create obstacle
                    optionsStruct = struct;
                    optionsStruct.component_options.info.creation_buffer = creation_buffer;
                    prop_obs = rtd.entity.BoxObstacle.makeBox(center, side_lengths, optionsStruct);

                    % test it
                    proposal_obj = prop_obs.collision.getCollisionObject(buffered=true);

                    % test it in the collision system
                    [randomizing, pairs] = self.collision_system.checkCollisionObject(proposal_obj);
                    t_cur = toc(start_tic);
                end
                % if it's good, we save the proposal_obj
                self.add_object(prop_obs, collision=prop_obs.collision.getCollisionObject, visual=prop_obs.visual);
            end

            % Create and add the goal
            self.goal_system = armour.deprecation.RandomArmConfigurationGoal(self.collision_system, self.agent);
            self.goal_system.reset();
            self.goal_system.createGoal();
            self.visual_system.addObjects(static=self.goal_system);

            % reset the agent
            self.agent.reset

            % redraw
            self.visual_system.redraw();
            
            self.simulation_state = 'READY';
        end
        function info = pre_step(self)
            self.simulation_state = 'PRE_STEP';
            
            % CALL PLANNER
            info = struct;

            % TODO setup custom event data to return the sim
            notify(self, 'PreStep')
        end
        function info = step(self)
            self.simulation_state = 'STEP';
            
            % Update entities
            agent_results = self.agent.update(self.simulation_timestep);
            
            % Update systems
            [collision, contactPairs] = self.collision_system.updateCollision(self.simulation_timestep);

            if collision
                disp("Collision Detected, Breakpoint!")
                pause
                disp("Continuing")
            end
            info.agent_results = agent_results;
            info.collision = collision;
            info.contactPairs = contactPairs;

            % TODO setup custom event data to return the sim
            notify(self, 'Step')
        end
        function info = post_step(self)
            self.simulation_state = 'POST_STEP';
            % Check if goal was achieved
            goal = self.goal_system.updateGoal(self.simulation_timestep);
            pause_requested = self.visual_system.updateVisual(self.simulation_timestep);
            info.goal = goal;
            info.pause_requested = pause_requested;

            % TODO setup custom event data to return the sim
            notify(self, 'PostStep')
        end
        function summary(self, options)
        end
        function run(self, options)
            arguments
                self armour.ArmourSimulation
                options.max_steps = 1e8
                options.max_time = Inf
                options.pre_step_callback cell = {}
                options.step_callback cell = {}
                options.post_step_callback cell = {}
                options.stop_on_goal = true
            end
            
            % Build the execution order
            execution_queue = [ {@(self)self.pre_step()},   ...
                                options.pre_step_callback,  ...
                                {@(self)self.step()},       ...
                                options.step_callback,      ...
                                {@(self)self.post_step()},  ...
                                options.post_step_callback ];

            steps = 0;
            start_tic = tic;
            t_cur = toc(start_tic);
            pause_time = 0;
            stop = false;
            while steps < options.max_steps && t_cur < options.max_time && ~stop
                % Iterate through all functions in the execution queue
                for fcn = execution_queue
                    info = fcn{1}(self);
                    % Automate logging here if wanted
                    stop = stop || (isfield(info, 'stop') && info.stop);
                    if options.stop_on_goal && isfield(info, 'goal') && info.goal
                        stop = true;
                        disp("Goal acheived!")
                    end
                    % Pause if requested
                    if isfield(info, 'pause_requested') && info.pause_requested
                        start_pause = tic;
                        keyboard
                        pause_time = pause_time + toc(start_pause);
                    end
                end
                steps = steps + 1;
                t_cur = toc(start_tic) - pause_time;
            end
        end
    end
end
