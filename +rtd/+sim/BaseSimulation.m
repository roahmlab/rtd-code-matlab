classdef BaseSimulation < rtd.util.mixins.NamedClass & handle
% BaseSimulation: Base class for all simulations
%
% Whenever creating a new simulation, it is recommended to inherit from
% this class. This class provides the basic functionality for running a
% simulation. It also provides a few helper functions for exporting and
% importing the world model. The setup and initialize functions are expected
% to be defined. The simulation must be put into the state of "READY" before
% any of the step functions can be called.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-20
%
% See also: rtd.sim.world.WorldModel, rtd.sim.types.SimulationState
%
% --- More Info ---
%

    % Some default properties that are useful for all simulations
    properties
        % Internal history of time for the overall simulation
        % This is used to verify synchrony of all entities and components
        simulation_time(:,1) double = 0;

        % Configuration state / step state of the simulation
        simulation_state rtd.sim.types.SimulationState = 'INVALID'

        % Historical log of the simulation
        simulation_log rtd.util.containers.VarLogger = rtd.util.containers.VarLogger.empty()

        % Container which holds the entities and systems relevant for the simulation
        world rtd.sim.world.WorldModel = rtd.sim.world.WorldModel.empty()
    end

    % Properties that must be defined by the user
    properties (Abstract)
        % Define this variable to specify what a complete step of the simulation should progress
        % forward in time by
        simulation_timestep(1,1) double
    end

    % Events that are triggered during the lifecycle of the simulation
    events
        % Triggered after pre-step operations, before step operations
        PreStep

        % Triggered after step operations, before post-step operations
        Step

        % Triggered after post-step operations
        PostStep
    end

    % Methods that must be defined by the user
    methods (Abstract)
        % This function should setup key entities and systems
        setup(self)

        % This function should run after setup to initialize everything to start
        initialize(self)
        
        % This function should imports the world model from an xml file
        import(self, filename)
        
        % saves and loads the simulation
        %save_checkpoint(self, filename)
        %load_checkpoint(self, filename)
    end

    % Optionally overridable function
    methods
        % Generate a default summary based on the simulation log
        function struct_out = summary(self)
            % Default implementation returns the consolidated log
            %
            % This function should be overridden by the user to
            % provide a summary with desired information as a struct
            %
            % Returns:
            %   struct_out: Struct of what happened during the simulation run
            %

            if ~isempty(self.simulation_log)
                struct_out = self.simulation_log.get;
            end
        end
    end

    % Internal methods for the lifecycle of the simulation
    % These methods are meant to be overridden by the user
    methods (Access = protected)
        % Execute before the overall step
        function info = pre_step_impl(self)
            % Default implementation does nothing
            %
            % This function is meant to be overridden by the user
            % to perform any pre-step operations
            %
            % Returns:
            %   info: Struct of information of what happened during the pre-step
            %

            info = struct;
        end

        % Execute all the updates needed for each step
        function info = step_impl(self)
            % Default implementation does nothing
            %
            % This function is meant to be overridden by the user
            % to perform any step operations
            %
            % Returns:
            %   info: Struct of information of what happened during the step
            %
            
            info = struct;
        end

        % Execute after each step
        function info = post_step_impl(self)
            % Default implementation does nothing
            %
            % This function is meant to be overridden by the user
            % to perform any post-step operations
            %
            % Returns:
            %   info: Struct of information of what happened during the post-step
            %
            
            info = struct;
        end
    end

    % Sealed methods for the lifecycle of the simulation
    methods (Sealed)
        function info = pre_step(self)
            % Execute before the overall step
            %
            % This is the function that the user calls to execute the
            % pre-step operations
            %
            % Returns:
            %   info: Struct of information of what happened during the pre-step
            %
            % Notifies:
            %   PreStep: Notifies all listeners that the pre-step has been executed
            %

            if self.simulation_state < "READY"
                error("Simulation not ready! Make sure to set the simulation_state to ready when ready!")
            end
            self.simulation_state = 'PRE_STEP';
            info = self.pre_step_impl();
            notify(self, 'PreStep')
        end

        function info = step(self)
            % Execute all the updates needed for each step
            %
            % This is the function that the user calls to execute the
            % step operations
            %
            % Returns:
            %   info: Struct of information of what happened during the step
            %
            % Notifies:
            %   Step: Notifies all listeners that the step has been executed
            %

            if self.simulation_state < "READY"
                error("Simulation not ready! Make sure to set the simulation_state to ready when ready!")
            end
            self.simulation_state = 'STEP';
            info = self.step_impl();
            notify(self, 'Step')
        end

        function info = post_step(self)
            % Execute after each step
            %
            % This is the function that the user calls to execute the
            % post-step operations. This function also updates the
            % simulation time and checks for synchronization of all entities
            % and systems.
            %
            % Returns:
            %   info: Struct of information of what happened during the post-step
            %
            % Notifies:
            %   PostStep: Notifies all listeners that the post-step has been executed
            %

            if self.simulation_state < "READY"
                error("Simulation not ready! Make sure to set the simulation_state to ready when ready!")
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

        function run(self, options)
            % Run the lifecycle of the simulation all at once
            %
            % This is the function that the user calls to execute the
            % entire lifecycle of the simulation if they don't want to
            % manually call each step. This function also provides the
            % ability to automatically log the simulation. It will run
            % until the simulation is stopped or the max number of steps
            % or max time is reached.
            %
            % Arguments:
            %   options: Keyword arguments. See below
            %
            % Keyword Arguments:
            %   max_steps: Maximum number of steps to run the simulation for. Default: 1e8
            %   max_time: Maximum amount of time to run the simulation for. Default: Inf
            %   pre_step_callback: Function handle to execute before each step. Default: {}
            %   step_callback: Function handle to execute during each step. Default: {}
            %   post_step_callback: Function handle to execute after each step. Default: {}
            %   stop_on_goal: Stop the simulation if the goal is reached. Default: true
            %   autolog: Automatically log the simulation. Default: false
            %

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
        function export(self, filename)
            % Export the world model to an xml
            %
            % This function exports the world model to an xml file. If
            % multiple world models are present, then each world model
            % will be exported to a separate xml file with the index
            % appended to the filename. If no filename is provided, then
            % the default filename will be the name of the simulation
            % class and the current date and time.
            %
            % Arguments:
            %   filename: Name of the file to export to. Default: ''
            %
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