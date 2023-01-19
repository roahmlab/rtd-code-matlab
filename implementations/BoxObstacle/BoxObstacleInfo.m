classdef BoxObstacleInfo < EntityInfo & mixins.Options & handle
    % Leftover Old Dependencies
    % load_robot_params by extension -> make it a class
    % create_pz_bounding_boxes
    % 
    % Notes
    % links should become an independently validatable class
    % joints should as well, or be a part of the links one
    
    properties
        % Default for 2D, but can be 3D
        dimension = 2
        
        % The x,y,z side lengths of the box
        side_lengths (1,:) double = [1,1]
        
        % Buffer for when randomizing during creation time
        creation_buffer (1,1) double = 0
        
        % Is this the base of a robot?
        % Exists for compat
        is_base_obstacle (1,1) logical = false
    end
    
    methods (Static)
        function options = defaultoptions()
            % Configurable options for this component
            options.dimension = [];
            options.side_lengths = [1, 1]; % arbitrary value
            options.creation_buffer = 0;
            options.is_base_obstacle = false;
        end
    end
    
    methods
        function self = BoxObstacleInfo(optionsStruct, options)
            arguments
                optionsStruct struct = struct()
                options.dimension
                options.side_lengths
                options.creation_buffer
                options.is_base_obstacle
            end
            self.mergeoptions(optionsStruct, options);
            
            % initialize
            self.reset();
        end
        
        % We set this here, but is it really necessary?
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.dimension
                options.side_lengths
                options.creation_buffer
                options.is_base_obstacle
            end
            options = self.mergeoptions(optionsStruct, options);
            
            % Validate
            if isempty(options.dimension)
                options.dimension = length(options.side_lengths);
            end
            if options.dimension ~= length(options.side_lengths)
                error("Box dimension must match side lengths!");
            end
            
            % Save
            self.dimension = options.dimension;
            self.side_lengths = options.side_lengths;
            self.creation_buffer = options.creation_buffer;
            self.is_base_obstacle = options.is_base_obstacle;
        end
    end
end