classdef Patch3dCollisionSystem < rtd.core.systems.SimulationSystem & rtd.core.mixins.NamedClass & rtd.core.mixins.Options & handle
    % Required inherited properties
    properties
        time = 0
        time_discretization = 0.1
        system_log = rtd.core.containers.VarLogger.empty()
    end
    % Additional properties we add
    properties
        static_objects (1,:) rtd.core.systems.patch3d_collision.Patch3dObject = rtd.core.systems.patch3d_collision.Patch3dObject.empty()
        dynamic_objects (1,:) rtd.core.systems.patch3d_collision.Patch3dDynamicObject = rtd.core.systems.patch3d_collision.Patch3dDynamicObject.empty()
    end
    
    % Default Options
    methods (Static)
        function options = defaultoptions()
            options.time_discretization = 0.1;
            options.log_collisions = false;
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end
    
    methods
        function self = Patch3dCollisionSystem(objects, optionsStruct, options)
            arguments
                objects.static_objects (1,:) rtd.core.systems.patch3d_collision.Patch3dObject = rtd.core.systems.patch3d_collision.Patch3dObject.empty()
                objects.dynamic_objects (1,:) rtd.core.systems.patch3d_collision.Patch3dDynamicObject = rtd.core.systems.patch3d_collision.Patch3dDynamicObject.empty()
                optionsStruct.options struct = struct()
                options.time_discretization
                options.log_collisions
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct.options, options);
            
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
                options.log_collisions
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct, options);
            
            % if we're going to log, set it up
            if options.log_collisions
                self.system_log = rtd.core.containers.VarLogger('contactPairs');
            end
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            % For collision checking
            self.time_discretization = options.time_discretization;
            
            % Clear all the stored objects
            self.static_objects = rtd.core.systems.patch3d_collision.Patch3dObject.empty();
            self.dynamic_objects = rtd.core.systems.patch3d_collision.Patch3dDynamicObject.empty();
        end
        
        function addObjects(self, objects)
            arguments
                self rtd.core.systems.patch3d_collision.Patch3dCollisionSystem
                objects.static (1,:) rtd.core.systems.patch3d_collision.Patch3dObject = rtd.core.systems.patch3d_collision.Patch3dObject.empty()
                objects.dynamic (1,:) rtd.core.systems.patch3d_collision.Patch3dDynamicObject = rtd.core.systems.patch3d_collision.Patch3dDynamicObject.empty()
            end
            % TODO fix assumptions
            self.static_objects = [self.static_objects, objects.static];
            self.dynamic_objects = [self.dynamic_objects, objects.dynamic];
        end
        
        function remove(self, object)
            arguments
                self rtd.core.systems.patch3d_collision.Patch3dCollisionSystem
            end
            arguments (Repeating)
                object (1,1)
            end
            
            % Get all uuids
            static_uuids = {self.static_objects.uuid};
            static_parent_uuids = {self.static_objects.parent_uuid};
            dynamic_uuids = {self.dynamic_objects.uuid};
            
            % final idx's to delete
            static_idxs = [];
            dynamic_idxs = [];
            
            % Check each uuid.
            for obj = object
                if isprop(obj, 'uuid')
                    uuid = obj{1}.uuid;
                elseif ischar(obj) || isstring(obj)
                    uuid = obj{1};
                else
                    error("Must be UUID string or character array, or the (dynamic) object to remove");
                end

                % Get the idxs
                static_idxs_obj = find(strcmp(static_uuids, uuid));
                static_idxs_obj_parents = find(strcmp(static_parent_uuids, uuid));
                static_idxs_obj = union(static_idxs_obj, static_idxs_obj_parents);
                dynamic_idxs_obj = find(strcmp(dynamic_uuids, uuid));
                
                % Save them to remove all at once
                static_idxs = union(static_idxs, static_idxs_obj);
                dynamic_idxs = union(dynamic_idxs, dynamic_idxs_obj);
            end
            % remove them
            self.static_objects(static_idxs) = [];
            self.dynamic_objects(dynamic_idxs) = [];
        end
        
        function [collision, contactPairs] = updateCollision(self, t_update)
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
            
            self.vdisp('Running collision check!', 'DEBUG');
            
            % Accumulate the return
            collision = false;
            contactPairs = [];
            for t_check = t_vec
                res = self.checkCollisionAtTime(t_check);
                collision = collision || res.num_pairs > 0;
                contactPairs = [contactPairs, res];
            end
            
            % Save the time change
            self.time = [self.time, t_vec];
            
            % Log if we want
            if ~isempty(self.system_log)
                self.system_log.add('contactPairs', contactPairs);
            end
        end
        
        function contactPairs = checkCollisionAtTime(self, time)
            % Lazy method. Check each dynamic object against the static
            % objects and each other
            % Adding some sort of space partitioning would be much better.
            
            % Create return structure
            contactPairs.time = time;
            contactPairs.num_pairs = 0;
            contactPairs.pairs = [];
            
            % Resolve the object volumes for each of the dynamic entities
            get_vol = @(obj) obj.getCollisionObject(time=time);
            resolved_dyn_objs = arrayfun(get_vol, self.dynamic_objects);
            
            % Iterate through the dynamic objects
            collision = false;
            for dyn_idx = 1:length(resolved_dyn_objs)
                % Check each dynamic object with each static object
                for obj = self.static_objects
                    [c, pair] = resolved_dyn_objs(dyn_idx).inCollision(obj);
                    contactPairs.pairs = [contactPairs.pairs; pair];
                    collision = collision || c;
                end
                % Check each dynamic object with each remaining dynamic
                % object
                for dyn_idx_2 = dyn_idx:length(resolved_dyn_objs)
                    [c, pair] = resolved_dyn_objs(dyn_idx).inCollision(resolved_dyn_objs(dyn_idx_2));
                    contactPairs.pairs = [contactPairs.pairs; pair];
                    collision = collision || c;
                end
            end
            % Update and output that a collision occured
            if collision
                contactPairs.num_pairs = size(contactPairs.pairs,1);
                msg = sprintf("Collision at t=%.2f detected!", time);
                self.vdisp(msg, 'ERROR');
                % Debug all collision pairs
                if self.verboseLevel < "DEBUG"
                    self.vdisp("Collision pairs are as follows", 'DEBUG');
                    for idx = 1:contactPairs.num_pairs
                        msg = sprintf("Collision detected between %s and %s", ...
                                        contactPairs.pairs{idx,1}, ...
                                        contactPairs.pairs{idx,2});
                        self.vdisp(msg, 'DEBUG');
                    end
                end
            end
        end
        
        function [collision, contactPairs] = checkCollisionObject(self, collision_obj)
            % Lazy method. Check collision object against the static
            % objects and dynamic objects (like a proposal object)
            % Adding some sort of space partitioning would be much better.
            
            % Create return structure
            contactPairs.time = self.time;
            contactPairs.num_pairs = 0;
            contactPairs.pairs = [];
            
            % Resolve the object volumes for each of the dynamic entities
            get_vol = @(entity)entity.getCollisionObject();
            resolved_dyn_objs = arrayfun(get_vol, self.dynamic_objects);
            
            % Iterate through the dynamic objects
            collision = false;
            % Check each static object
            for obj = self.static_objects
                [c, pair] = collision_obj.inCollision(obj);
                contactPairs.pairs = [contactPairs.pairs; pair];
                collision = collision || c;
            end
            % Check each dynamic object
            for dyn_idx = 1:length(resolved_dyn_objs)
                [c, pair] = collision_obj.inCollision(resolved_dyn_objs(dyn_idx));
                contactPairs.pairs = [contactPairs.pairs; pair];
                collision = collision || c;
            end
            % Update number of collision pairs for the output
            contactPairs.num_pairs = size(contactPairs.pairs,1);
        end
    end
end