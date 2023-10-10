classdef WaitrSimulation < armour.ArmourSimulation & handle

    % Simulation Methods
    methods
        function initialize(self)
            % Initialize the simulation
            %
            % This function will initialize the simulation. This needs to
            % be called after setup and will ready the simulation for
            % stepping.
            %
            arguments
                self(1,1) waitr.WaitrSimulation
            end

            if self.simulation_state > "INITIALIZING"
                error("This simulation currently does not support reinitialization without resetup");
            end
            self.simulation_state = 'INITIALIZING';

            % A lot of temporaries
            timeout = 10;
            % Create a random start (assuming no obstacles)
            % randomizing = true;
            % start_tic = tic;
            % t_cur = toc(start_tic);
            % while randomizing && t_cur <= timeout
            %     self.agent.state.random_init(save_to_options=true);
            %     proposal_obj = self.agent.collision.getCollisionObject();

            %     % test it in the collision system
            %     [randomizing, ~] = self.collision_system.checkCollisionObject(proposal_obj);
            %     t_cur = toc(start_tic);
            % end
            self.agent.state.reset(initial_position=[0, -pi/2, 0, 0, 0, 0, 0]);

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
%             self.goal_system.createGoal([2.19112372555967;0.393795848789382;-2.08886547149797;-1.94078143810946;-1.82357815033695;-1.80997964933365;2.12483409695310]);
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
end