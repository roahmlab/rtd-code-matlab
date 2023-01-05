classdef BoxPatchCollision < Patch3dDynamicObject & NamedClass & handle
    
    
    % Leftover Old Dependencies
    % make_cuboid_for_patch
    % make_box
    
    properties
        box_info BoxObstacleInfo = BoxObstacleInfo.empty()
        box_state GenericStateComponent = GenericStateComponent.empty()
        
        collision_patch_data(1,1) struct
    end
    
    
    
    methods
        function self = BoxPatchCollision(box_info,box_state_component, verbose_level, name)
            arguments
                box_info BoxObstacleInfo
                box_state_component GenericStateComponent
                verbose_level = LogLevel.INFO
                name = ''
            end
            self.box_info = box_info;
            self.box_state = box_state_component;
            
            self.set_vdisplevel(verbose_level);
            self.name = name;
            
            self.reset()
        end
        
        function reset(self, ~)
            self.create_collision_check_patch_data()
        end
        
        % collision check data
        function create_collision_check_patch_data(self)
            dim = self.box_info.dimension;
            side_len = self.box_info.side_lengths;
            buff_side_len = self.box_info.side_lengths + 2*self.box_info.creation_buffer;
            center = zeros(dim,1) ;
            
            switch dim
                case 2
                    [F,V] = make_box(side_len,center) ;
                    [~,V_buff] = make_box(buff_side_len,center) ;
                    
                    % Expand dims here for compat with Patch3d
                    zero_col = zeros(size(V,1),1);
                    V = [V, zero_col];
                    V_buff = [V_buff, zero_col];
                case 3
                    [~,V] = make_cuboid_for_patch(side_len(1),side_len(2),side_len(3),center) ;
                    [~,V_buff] = make_cuboid_for_patch(buff_side_len(1),buff_side_len(2),buff_side_len(3),center) ;
                    F = convhull(V);
            end
            
            self.collision_patch_data.faces = F ;
            self.collision_patch_data.vertices = V ;
            self.collision_patch_data.vertices_buffered = V_buff ;
        end
        
        % get collision check volume
        function out = getCollisionObject(self,options)
            arguments
                self
                options.buffered = false
                options.state = self.box_state.get_state()
                options.time = []
            end
            % get position given or the time to query.
            state = options.state;
            if ~isempty(options.time)
                state = self.arm_state.get_state(options.time);
            end
            
            % Compat for Patch3d
            shift = state.state.';
            if self.box_info.dimension == 2
                shift = [shift, 0];
            end
            
            F = self.collision_patch_data.faces;
            % select and shift the vertices
            if options.buffered
                V = self.collision_patch_data.vertices_buffered + shift;
            else
                V = self.collision_patch_data.vertices + shift;
            end
            
            % Create the Patch3dObject object
            uuid = self.box_info.uuid;
            out = Patch3dObject(uuid, F, V);
            % add centers
            out.centers = shift;
        end
    end
end