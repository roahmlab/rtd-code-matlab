classdef PatchVisualSystem < rtd.sim.systems.SimulationSystem & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle
    % TODO incorporate axes so that it's actually plotting everything
    % without deleting stuff or requiring hold on
    
    % Required inherited properties
    properties
        time = 0
        time_discretization = 0.1
        system_log = rtd.util.containers.VarLogger.empty()
    end
    % Additional properties we add
    properties
        static_objects (1,:) rtd.sim.systems.patch_visual.PatchVisualObject = rtd.sim.systems.patch_visual.PatchVisualObject.empty()
        dynamic_objects (1,:) rtd.sim.systems.patch_visual.PatchVisualObject = rtd.sim.systems.patch_visual.PatchVisualObject.empty()
        draw_time = 0.05
        figure_handle
        pause_requested = false;
        enable_camlight
    end
    
    % Default Options
    methods (Static)
        function options = defaultoptions()
            options.time_discretization = 0.1;
            %options.log_collisions = false;
            options.enable_camlight = false;
            options.xlim = [];
            options.ylim = [];
            options.zlim = [];
            options.view = 3;
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end
    
    methods
        function self = PatchVisualSystem(objects, optionsStruct, options)
            arguments
                objects.static_objects (1,:) rtd.sim.systems.patch_visual.PatchVisualObject = rtd.sim.systems.patch_visual.PatchVisualObject.empty()
                objects.dynamic_objects (1,:) rtd.sim.systems.patch_visual.PatchVisualObject = rtd.sim.systems.patch_visual.PatchVisualObject.empty()
                optionsStruct.options struct = struct()
                options.time_discretization
                options.enable_camlight
                options.xlim
                options.ylim
                options.zlim
                options.view
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct.options, options);
            
            % Reset first
            self.reset()
            
            % add static or dynamic objects if provided
            self.addObjects(static=objects.static_objects, dynamic=objects.dynamic_objects);

            % Trigger a redraw
            self.redraw()
        end
            
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct.options struct = struct()
                options.time_discretization
                options.enable_camlight
                options.xlim
                options.ylim
                options.zlim
                options.view
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct.options, options);
            
            % if we're going to log, set it up
            %if options.log_collisions
            %    self.system_log = rtd.util.containers.VarLogger('contactPairs');
            %end
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            % For collision checking
            self.time_discretization = options.time_discretization;
            
            % Clear all the stored objects
            self.static_objects = rtd.sim.systems.patch_visual.PatchVisualObject.empty();
            self.dynamic_objects = rtd.sim.systems.patch_visual.PatchVisualObject.empty();

            % Set camlight flag
            self.enable_camlight = options.enable_camlight;
            
            % Create a new figure if the current figure handle is bad.
            self.validateOrCreateFigure()

            % Reset pause flag
            self.pause_requested = false;
        end
        
        function validateOrCreateFigure(self)
            % Create a new figure if the current figure handle is bad.
            if isempty(self.figure_handle) || ...
                    ~isvalid(self.figure_handle) || ...
                    ~isgraphics(self.figure_handle)
                % Save the prior figure
                prev_fig = get(groot,'CurrentFigure');
                % Setup the new figure
                self.figure_handle = figure('Name',[self.name, ' - ', self.classname]);
                set(self.figure_handle,'KeyPressFcn',@self.keypress);
                set(self.figure_handle,'GraphicsSmoothing','off');
                % Restore the prior figure
                try
                set(groot, 'CurrentFigure', prev_fig);
                catch
                end
            end
        end    
        
        function addObjects(self, objects)
            arguments
                self rtd.sim.systems.patch_visual.PatchVisualSystem
                objects.static (1,:) rtd.sim.systems.patch_visual.PatchVisualObject = rtd.sim.systems.patch_visual.PatchVisualObject.empty()
                objects.dynamic (1,:) rtd.sim.systems.patch_visual.PatchVisualObject = rtd.sim.systems.patch_visual.PatchVisualObject.empty()
            end
            % Merge the objects in
            self.static_objects = [self.static_objects, objects.static];
            self.dynamic_objects = [self.dynamic_objects, objects.dynamic];
        end
        
        function remove(self, object)
            arguments
                self rtd.sim.systems.patch_visual.PatchVisualSystem
            end
            arguments (Repeating)
                object (1,1)
            end
            
            % final idx's to delete
            static_idxs = [];
            dynamic_idxs = [];
            
            % Check each uuid.
            for obj = object
                % Get the idxs
                static_idxs_obj = find(self.static_objects == obj);
                dynamic_idxs_obj = find(self.dynamic_objects == obj);
                
                % Save them to remove all at once
                static_idxs = union(static_idxs, static_idxs_obj);
                dynamic_idxs = union(dynamic_idxs, dynamic_idxs_obj);
            end
            % remove them
            self.static_objects(static_idxs) = [];
            self.dynamic_objects(dynamic_idxs) = [];
        end
        
        function request_pause = updateVisual(self, t_update)
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
            
            self.vdisp('Running visualization!', 'DEBUG');
            
            % Save the prior figure
            prev_fig = get(groot,'CurrentFigure');
            
            % set the active figure
            try
                set(groot, 'CurrentFigure', self.figure_handle)
    
                % plot each of the times requested
                for t_plot = t_vec
                    for obj = self.dynamic_objects
                        obj.plot(time=t_plot)
                    end
                    pause(self.draw_time)
                end
            catch
                % If reopen on close is enabled
            end

            % Restore the prior figure
            try
            set(groot, 'CurrentFigure', prev_fig);
            catch
            end
            
            % Save the time change
            self.time = [self.time, t_vec];
            
            % Say if pause was requested
            request_pause = self.pause_requested;
            self.pause_requested = false;
        end
        
        function redraw(self, time, options)
            arguments
                self
                time = self.time(end)
                options.xlim = []
                options.ylim = []
                options.zlim = []
                options.view = []
                options.reset_view = false
            end
            
            % if the figure handle is deleted, recreate it
            self.validateOrCreateFigure()
            % Save the prior figure
            prev_fig = get(groot,'CurrentFigure');
            % set the active figure
            set(groot, 'CurrentFigure', self.figure_handle);

            % Try to save the limits if possible
            try
                ax = get(self.figure_handle, 'CurrentAxes');
                if strcmp(ax.XLimMode, 'manual') ...
                        && isempty(options.xlim)
                    options.xlim = ax.XLim;
                end
                if strcmp(ax.YLimMode, 'manual') ...
                        && isempty(options.ylim)
                    options.ylim = ax.YLim;
                end
                if strcmp(ax.ZLimMode, 'manual') ...
                        && isempty(options.zlim)
                    options.zlim = ax.ZLim;
                end
            catch
            end

            % If provided, use the view
            if options.reset_view
                clf
                if iscell(self.getoptions.view)
                    view(self.getoptions.view{:})
                else
                    view(self.getoptions.view)
                end
            elseif ~isempty(options.view)
                clf
                if iscell(options.view)
                    view(options.view{:})
                else
                    view(options.view)
                end
            else
                % Save the camera view if possible
                try
                    ax = get(self.figure_handle, 'CurrentAxes');
                    [az, el] = view(ax);
                    % clear and reset view
                    clf;view(az, el)
                catch
                    clf
                    if iscell(self.getoptions.view)
                        view(self.getoptions.view{:})
                    else
                        view(self.getoptions.view)
                    end
                end
            end

            axis equal;grid on
            % Set limits
            if ~options.reset_view
                if ~isempty(options.xlim)
                    xlim(options.xlim)
                elseif ~isempty(self.getoptions.xlim)
                    xlim(self.getoptions.xlim)
                end
                if ~isempty(options.ylim)
                    ylim(options.ylim)
                elseif ~isempty(self.getoptions.ylim)
                    ylim(self.getoptions.ylim)
                end
                if ~isempty(options.zlim)
                    zlim(options.zlim)
                elseif ~isempty(self.getoptions.zlim)
                    zlim(self.getoptions.zlim)
                end
            end

            if self.enable_camlight
                camlight
            end

            for obj = self.static_objects
                obj.plot(time=time)
            end
            for obj = self.dynamic_objects
                obj.plot(time=time)
            end
            drawnow limitrate
            %pause(self.draw_time)
            % Restore the prior figure
            try
            set(groot, 'CurrentFigure', prev_fig);
            catch
            end
        end
        
        function animate(self, options)
            arguments
                self
                options.t_span = [0, self.time(end)]
                options.time_discretization = self.time_discretization
                options.pause_time = []
                options.xlim = []
                options.ylim = []
                options.zlim = []
                options.view = []
                options.reset_view = false
            end
            
            if isempty(options.pause_time)
                options.pause_time = options.time_discretization;
            end
            
            start_time = options.t_span(1);
            end_time = options.t_span(2);
            t_vec = start_time:options.time_discretization:end_time;
            proc_time_start = tic;
            
            % Redraw everything
            self.redraw(0, ...
                        xlim=options.xlim, ...
                        ylim=options.ylim, ...
                        zlim=options.zlim, ...
                        view=options.view, ...
                        reset_view=options.reset_view);

            % Save the prior figure
            prev_fig = get(groot,'CurrentFigure');
            % set the active figure
            set(groot, 'CurrentFigure', self.figure_handle);
            
            % Animate the dynamic stuff
            for t_plot = t_vec
                proc_time = toc(proc_time_start);
                pause_time = options.pause_time - proc_time;
                proc_time_start = tic;
                if pause_time < 0
                    self.vdisp(['Warning, animation lagging by ', num2str(-pause_time), 's!'], 'WARN');
                end
                drawnow
                pause(max(pause_time, 0));
                for obj = self.dynamic_objects
                    obj.plot(time=t_plot)
                end
                if self.pause_requested
                    self.vdisp("Pausing", 'INFO');
                    keyboard
                    self.vdisp("Resuming", 'INFO');
                    self.pause_requested = false;
                end
            end
            % Restore the prior figure
            try
            set(groot, 'CurrentFigure', prev_fig);
            catch
            end
        end

        function keypress(self, src, event)
            if event.Character == "p" && ~self.pause_requested
                self.vdisp("Pause requested", 'INFO');
                self.pause_requested = true;
            end
            if event.Character == "r"
                self.vdisp("Redraw requested", 'INFO');
                self.redraw();
            end
        end

        function close(self)
            % try to close
            try
                close(self.figure_handle)
            catch
            end
        end

    end

    methods (Static)
        function [time_discretization, pause_time] = get_discretization_and_pause(framerate, speed)
            arguments
                framerate (1,1) double = 30
                speed (1,1) double = 1
            end
            pause_time = 1 / framerate;
            time_discretization = speed / framerate;
        end
    end
end