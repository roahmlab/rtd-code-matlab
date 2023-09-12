classdef BoxObstacleZonotope < rtd.util.mixins.NamedClass & handle
    
    
    % Leftover Old Dependencies
    % make_cuboid_for_patch
    % make_box
    
    properties
        box_info rtd.entity.box_obstacle.BoxObstacleInfo = rtd.entity.box_obstacle.BoxObstacleInfo.empty()
        box_state rtd.entity.components.GenericEntityState = rtd.entity.components.GenericEntityState.empty()
        
        base_zonotope
    end
    
    methods
        function self = BoxObstacleZonotope(box_info,box_state_component)
            self.box_info = box_info;
            self.box_state = box_state_component;
            
            %self.reset();
        end
        
        function reset(self)
            center = zeros(self.box_info.dimension,1);
            self.base_zonotope = zonotope([center, diag(self.box_info.side_lengths./2)]);
        end
        
        function zono = get_zonotope(self,options)
            arguments
                self
                options.state = self.box_state.get_state()
                options.time = []
            end
            state = options.state;
            if ~isempty(options.time)
                state = self.box_state.get_state(options.time);
            end
            
            zono = self.base_zonotope + state.state_data;
        end
    end
end