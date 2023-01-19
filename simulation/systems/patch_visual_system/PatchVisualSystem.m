classdef PatchVisualSystem < SimulationSystem & mixins.NamedClass & mixins.Options & handle
    % TODO incorporate axes so that it's actually plotting everything
    % without deleting stuff or requiring hold on
    
    % Required inherited properties
    properties
        time = 0
        time_discretization = 0.1
        system_log = containers.VarLogger.empty()
    end
    % Additional properties we add
    properties
        static_objects (1,:) PatchVisualObject = PatchVisualObject.empty()
        dynamic_objects (1,:) PatchVisualObject = PatchVisualObject.empty()
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
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end
    
    methods
        function self = PatchVisualSystem(objects, optionsStruct, options)
            arguments
                objects.static_objects (1,:) PatchVisualObject = PatchVisualObject.empty()
                objects.dynamic_objects (1,:) PatchVisualObject = PatchVisualObject.empty()
                optionsStruct.optionsStruct struct = struct()
                options.time_discretization
                options.enable_camlight
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct.optionsStruct, options);
            
            % Reset first
            self.reset()
            
            % add static or dynamic objects if provided
            self.addObjects(static=objects.static_objects, dynamic=objects.dynamic_objects);
        end
            
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.time_discretization
                options.enable_camlight
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct, options);
            
            % if we're going to log, set it up
            %if options.log_collisions
            %    self.system_log = containers.VarLogger('contactPairs');
            %end
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            % For collision checking
            self.time_discretization = options.time_discretization;
            
            % Clear all the stored objects
            self.static_objects = PatchVisualObject.empty();
            self.dynamic_objects = PatchVisualObject.empty();

            % Set camlight flag
            self.enable_camlight = options.enable_camlight;

            % Create a new figure if the current figure handle is bad.
            if isempty(self.figure_handle) || ...
                    ~isvalid(self.figure_handle) || ...
                    ~isgraphics(self.figure_handle)
                self.figure_handle = figure('Name',[self.name, ' - ', self.classname]);
                view(3);
                set(self.figure_handle,'KeyPressFcn',@self.set_pause);
            end

            % Reset pause flag
            self.pause_requested = false;
        end
        
        function addObjects(self, objects)
            arguments
                self PatchVisualSystem
                objects.static (1,:) PatchVisualObject = PatchVisualObject.empty()
                objects.dynamic (1,:) PatchVisualObject = PatchVisualObject.empty()
            end
            % Merge the objects in
            self.static_objects = [self.static_objects, objects.static];
            self.dynamic_objects = [self.dynamic_objects, objects.dynamic];
        end
        
        function remove(self, object)
            arguments
                self PatchVisualSystem
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
            
            % set the active figure
            set(0, 'CurrentFigure', self.figure_handle)

            % plot each of the times requested
            for t_plot = t_vec
                for obj = self.dynamic_objects
                    obj.plot(time=t_plot)
                end
                pause(self.draw_time)
            end
            
            % Save the time change
            self.time = [self.time, t_vec];
            
            % Say if pause was requested
            request_pause = self.pause_requested;
            self.pause_requested = false;
        end
        
        function redraw(self, time)
            arguments
                self
                time = self.time(end)
            end
            % set the active figure
            set(0, 'CurrentFigure', self.figure_handle)
            % Save the camera view
            try
                ax = get(self.figure_handle, 'CurrentAxes');
                [az, el] = view(ax);
                % clear and reset view
                clf;view(az, el)
            catch
                clf;view(3)
            end
            axis equal;grid on
            if self.enable_camlight
                camlight
            end

            for obj = self.static_objects
                obj.plot(time=time)
            end
            for obj = self.dynamic_objects
                obj.plot(time=time)
            end
            pause(self.draw_time)
        end
        
        function animate(self, options)
            arguments
                self
                options.t_span = [0, self.time(end)]
                options.time_discretization = self.time_discretization
                options.pause_time = []
            end
            
            if isempty(options.pause_time)
                options.pause_time = options.time_discretization;
            end
            
            start_time = options.t_span(1);
            end_time = options.t_span(2);
            t_vec = start_time:options.time_discretization:end_time;
            proc_time_start = tic;
            
            % Redraw everything
            self.redraw(0);
            
            % Animate the dynamic stuff
            for t_plot = t_vec
                proc_time = toc(proc_time_start);
                pause_time = options.pause_time - proc_time;
                proc_time_start = tic;
                if pause_time < 0
                    self.vdisp(['Warning, animation lagging by ', num2str(-pause_time), 's!'], 'WARN');
                end
                pause(max(pause_time, 1e-8));
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
        end

        function set_pause(self, src, event)
            if event.Character == "p" && ~self.pause_requested
                self.vdisp("Pause requested", 'INFO');
                self.pause_requested = true;
            end
        end
    end
end