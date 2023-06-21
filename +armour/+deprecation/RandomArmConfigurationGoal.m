classdef RandomArmConfigurationGoal < rtd.sim.systems.patch_visual.PatchVisualObject & rtd.sim.systems.SimulationSystem & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle
    % Required properties from SimulationSystem
    % This class will be replaced soon with a agent-specific component to
    % better handle goal checking.
    properties
        time = 0
        time_discretization = 0.1
        system_log = rtd.util.containers.VarLogger.empty()
    end
    % Required properties from PatchVisualObject
    properties
        plot_data = struct
    end
    % Extra properties for this system
    properties
        collision_system
        arm_agent
        
        start_position
        min_dist_start_to_goal
        goal_creation_timeout
        goal_position
        goal_radius
        
        patch_style
        link_plot_data
    end
    
    methods (Static)
        function options = defaultoptions()
            options.time_discretization = 0.1;
            options.face_color = [0 1 0];
            options.face_opacity = 0.1;
            options.edge_color = [0 1 0];
            options.edge_opacity = 0.3;
            options.edge_style = '--';
            options.start_position = [];
            options.goal_position = [];
            options.min_dist_start_to_goal = [];
            options.goal_creation_timeout = 10;
            options.goal_radius = pi/30;
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end
    methods
        function self = RandomArmConfigurationGoal(collision_system,arm_agent,optionsStruct,options)
            arguments
                collision_system
                arm_agent
                optionsStruct.options struct = struct()
                options.time_discretization
                options.face_color
                options.face_opacity
                options.edge_color
                options.edge_opacity
                options.edge_width
                options.start_position
                options.goal_position
                options.min_dist_start_to_goal
                options.goal_creation_timeout
                options.goal_radius
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct.options, options);
            
            self.collision_system = collision_system;
            self.arm_agent = arm_agent;
            
            %self.reset();
        end
        
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct.options struct = struct()
                options.time_discretization
                options.face_color
                options.face_opacity
                options.edge_color
                options.edge_opacity
                options.edge_width
                options.start_position
                options.goal_position
                options.min_dist_start_to_goal
                options.goal_creation_timeout
                options.goal_radius
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct.options, options);
            self.time_discretization = options.time_discretization;
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            self.patch_style.FaceColor = options.face_color;
            self.patch_style.FaceAlpha = options.face_opacity;
            self.patch_style.EdgeColor = options.edge_color;
            self.patch_style.EdgeAlpha = options.edge_opacity;
            self.patch_style.LineStyle = options.edge_style;
            
            % other extra
            self.start_position = options.start_position;
            self.min_dist_start_to_goal = options.min_dist_start_to_goal;
            self.goal_creation_timeout = options.goal_creation_timeout;
            self.goal_radius = options.goal_radius;
            
            if isempty(self.start_position)
                self.start_position = self.arm_agent.state.get_state().q;
            end
            if isempty(self.min_dist_start_to_goal)
                pos_limits = [self.arm_agent.info.joints.position_limits];
                % set any joint limits that are +Inf to pi and -Inf to -pi
                infs = isinf(pos_limits);
                pos_limits(1,infs(1,:)) = -pi;
                pos_limits(2,infs(2,:)) = +pi;
                % Get the range and use that to compute the min_dist
                joint_ranges = diff(pos_limits,[],1);
                self.min_dist_start_to_goal = norm(0.25*joint_ranges);
            end
            
            % set the goal if provided
            if ~isempty(options.goal_position)
                self.createGoal(options.goal_position)
            end
            
            self.plot_data.links = [];
        end
        
        function goal = updateGoal(self, t_update)
            % Perform the collision check for t_update
            % Why is this update passed in as part of the function?
            % It means we can split the system up as much as we want, say
            % one step of the simulation is 4 seconds, but we want to call
            % all the systems at .5 second update steps. This makes that
            % possible.
            % We can call system 1 and 2 alternatingly until 4 seconds.
            % TODO: fix eventual discretization bug
            % nevermind, instead ensure each class has a time element and
            % ensure synchrony in the simulator. If synchronization is
            % lost, request a check to the discretization values and step
            % values.
            start_time = self.time(end)+self.time_discretization;
            end_time = self.time(end)+t_update;
            t_vec = start_time:self.time_discretization:end_time;
            
            self.vdisp('Running goal check!', 'DEBUG');
            
            % Accumulate the return
            goal = false;
            get_pos = @(t)self.arm_agent.state.get_state(t).q;
            for t_check = t_vec
                goal = goal || all(abs(get_pos(t_check) - self.goal_position) <= self.goal_radius, 'all');
            end
            
            % Save the time change
            self.time = [self.time, t_vec];
        end
        
        function createGoal(self, position)
            arguments
                self armour.deprecation.RandomArmConfigurationGoal
                position = []
            end
            
            % If we are given a position, just set that and call it a day
            if ~isempty(position)
                self.goal_position = position;
                self.resetVisual();
                return
            end
            
            % Check if the system is already active
            if self.time ~= 0
                error("Can't recreate goal for active system!")
            end

            proposed_goal = [] ;
            proposed_goal_dist = 0;
            start_tic = tic ;
            t_cur = toc(start_tic);
            
            while proposed_goal_dist < self.min_dist_start_to_goal && ...
                    t_cur <= self.goal_creation_timeout
                % update time
                t_cur = toc(start_tic);
                
                % NOTE: this will overwrite the arm position
                % TODO: consider isolating random state from random init
                % Propose a position for the arm & get the collision object
                self.arm_agent.state.random_init();
                proposal_obj = self.arm_agent.collision.getCollisionObject();

                % test it in the collision system
                [collision, pairs] = self.collision_system.checkCollisionObject(proposal_obj);
                % Restart if in collision
                if collision
                    continue
                end

                % compute distance to the goal & update time
                proposed_goal = self.arm_agent.state.get_state().q;
                proposed_goal_dist = norm(self.start_position - proposed_goal);
            end
            if isempty(proposed_goal)
                self.vdisp('Failed to find collision free goal! Using random goal', 'GENERAL');
                % Position is already randomized
                proposed_goal = self.arm_agent.state.get_state().q;
            end
            % save and update
            self.goal_position = proposed_goal;
            self.resetVisual();
            % Reset the arm back to the desired start
            self.arm_agent.state.reset(initial_position=self.start_position);
        end
        
        function resetVisual(self)
            % Generate a visual
            prev_fig = get(groot,'CurrentFigure');
            temp_fig = figure;
            self.arm_agent.visual.plot_links(self.goal_position);
            % Extract the data to plot
            faces = {self.arm_agent.visual.plot_data.links.Faces};
            vertices = {self.arm_agent.visual.plot_data.links.Vertices};
            self.link_plot_data.faces = faces;
            self.link_plot_data.vertices = vertices;
            % restore visual
            close(temp_fig);
            set(groot,'CurrentFigure',prev_fig);
        end
        
        function plot(self,options)
            arguments
                self
                options.time % ignored
            end
            
            % update if needed
            if ~self.isPlotDataValid('links')
                n_links = self.arm_agent.info.n_links_and_joints;
                link_array = gobjects(1, n_links);
                for idx = 1:n_links
                    link_array(idx) = patch(...
                        'Faces',    self.link_plot_data.faces{idx},...
                        'Vertices', self.link_plot_data.vertices{idx},...
                         self.patch_style);
                end
                self.plot_data.links = link_array ;
            end
        end
    end
end