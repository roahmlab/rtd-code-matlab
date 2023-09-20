classdef BaseSimulation < rtd.util.mixins.NamedClass & handle
    properties
        simulation_time = 0;
        simulation_state rtd.sim.types.SimulationState = 'INVALID'
        simulation_log rtd.util.containers.VarLogger = rtd.util.containers.VarLogger.empty()
        world rtd.sim.world.WorldModel = rtd.sim.world.WorldModel.empty()
    end
    properties (Abstract)
        simulation_timestep
    end
    methods (Abstract)
        % add some object to the simulation
        %add_object(self, object)
        % setup key entities and systems
        setup(self)
        % initialize everything to start
        initialize(self)
        
        % Imports the world model from an xml file
        import(self, filename)
        
        % saves and loads the simulation
        %save_checkpoint(self, filename)
        %load_checkpoint(self, filename)
    end
    methods
        % Execute before the overall step
        function info = pre_step_impl(self)
            info = struct;
        end
        % Execute all the updates needed for each step
        function info = step_impl(self)
            info = struct;
        end
        % Execute after each step
        function info = post_step_impl(self)
            info = struct;
        end
        % Generate a default summary based on the simulation log
        function struct_out = summary(self)
            if ~isempty(self.simulation_log)
                struct_out = self.simulation_log.get;
            end
        end
    end
    methods (Sealed)
        function info = pre_step(self)
            if self.simulation_state < "READY"
                error("Simulation not ready!")
            end
            self.simulation_state = 'PRE_STEP';
            info = self.pre_step_impl();
            notify(self, 'PreStep')
        end
        function info = step(self)
            if self.simulation_state < "READY"
                error("Simulation not ready!")
            end
            self.simulation_state = 'STEP';
            info = self.step_impl();
            notify(self, 'Step')
        end
        function info = post_step(self)
            if self.simulation_state < "READY"
                error("Simulation not ready!")
            end
            self.simulation_state = 'POST_STEP';
            info = self.post_step_impl();
            
            % Notify before validation
            notify(self, 'PostStep')

            % Update simulation time and make sure everything is in sync
            self.simulation_time(end+1) = self.simulation_time(end) + self.simulation_timestep;
            in_sync = false(length(self.world), 1);
            for idx=1:length(self.world)
                in_sync(idx) = self.world(idx).checkTimeSync(self.simulation_time(end));
            end
            if ~isempty(in_sync) && ~all(in_sync)
                warning("Simulation synchronization has been lost!")
            end
            info.simulation_poststep_time = self.simulation_time(end);
            info.simulation_in_sync = in_sync;
        end
        % Run the lifecycle
        % Max iterations or max length is embedded in this.
        function run(self, options)
            arguments
                self rtd.sim.BaseSimulation
                options.max_steps = 1e8
                options.max_time = Inf
                options.pre_step_callback cell = {}
                options.step_callback cell = {}
                options.post_step_callback cell = {}
                options.stop_on_goal = true
                options.autolog = false
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
                    if options.autolog && ~isempty(self.simulation_log)
                        self.simulation_log.addStruct(info);
                    end
                end
                steps = steps + 1;
                t_cur = toc(start_tic) - pause_time;
            end
        end
    end
    methods
        % Exports the world model to an xml
        function export(self, filename)
            arguments
                self(1,1) rtd.sim.BaseSimulation
                filename {mustBeTextScalar} = ''
            end
            if self.simulation_state < "READY"
                error("Unable to export simulation. Simulation hasn't been initialized!");
            end
            if isempty(filename)
                filename = [self.classname, '-', ...
                    char(datetime("now","Format",'yyyyMMdd''T''HHmmss'))];
            else
                % Strip the xml extension if provided.
                [path, name, extension] = fileparts(filename);
                if ~strcmpi(extension, '.xml')
                    name = [name, extension];
                end
                filename = fullfile(path, name);
            end
            if length(self.world) == 1
                self.world.export([filename, '.xml']);
            else
                for idx=1:length(self.world)
                    self.world.export([filename, '-', num2str(idx), '.xml']);
                end
            end
        end
    end
end