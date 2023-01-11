classdef WorldModel < handle
    %WORLDMODEL A model of the world used by RTD
    %   Similar to RobotModel, in that it contains a single initialization
    %   of WorldInfo, and then will create atomic instances of WorldState
    %   as needed as well as evolve world dynamics/predictions.
    
    properties
        agent
        obstacle
    end
    methods (Static)
        function world = import(filename)
        end
    end
    methods
        % Add object
        function addObject(self, object, options)
            arguments
                self WorldModel
                object
                options.type {mustBeMember(options.type,{'obstacle','agent'})} = 'obstacle'
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
                all_names = [fieldnames(self.agent); fieldnames(self.obstacle)];
                id = sum(cell2mat(regexp(all_names, search_string)));
                name = [char(name), num2str(id)];
            end
            object.name = name;
            self.world.(type).(name) = object;
            
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
                self.obstacles = [self.obstacles, box_obstacle_zonotope('center', object.state.state, 'side_lengths', object.info.side_lengths)];
                if ~isempty(options.collision)
                    self.collision_system.addObjects(static=options.collision);
                end
                % Add the visualization component provided to the visual
                % system
                if ~isempty(options.visual)
                    self.visual_system.addObjects(static=options.visual);
                end
            end
        end
        function export(self, filename)
        end
    end
end

