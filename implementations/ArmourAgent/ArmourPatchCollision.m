classdef ArmourPatchCollision < Patch3dDynamicObject & rtd.mixins.NamedClass & handle
    
    
    % Leftover Old Dependencies
    % make_cuboid_for_patch
    % make_ellipsoid_for_patch
    % make_cylinder_for_patch
    % check_if_plot_is_available - This is probably a major hog!
    
    properties
        arm_info ArmourAgentInfo = ArmourAgentInfo.empty()
        arm_state ArmourAgentState = ArmourAgentState.empty()
        kinematics ArmKinematics = ArmKinematics.empty()
        
        collision_patch_data(1,1) struct
    end
    
    
    
    methods
        function self = ArmourPatchCollision(arm_info,arm_state_component,kinematics_component, verbose_level, name)
            arguments
                arm_info ArmourAgentInfo
                arm_state_component ArmourAgentState
                kinematics_component ArmKinematics
                verbose_level = 'INFO'
                name = ''
            end
            self.arm_info = arm_info;
            self.arm_state = arm_state_component;
            self.kinematics = kinematics_component;
            
            self.set_vdisplevel(verbose_level);
            self.name = name;
            
            self.reset()
        end
        
        function reset(self, ~)
            self.create_collision_check_patch_data()
        end
        
        % collision check data
        function create_collision_check_patch_data(self)
            % A.create_collision_check_patch_data()
            %
            % Create a cell array of faces vectors (each 1-by-NF) and a
            % cell array of vertices arrays (each NV-by-2) to be used for
            % collision checking.
            
            
            %assert(self.arm_info.dimension == 3)
            % set up cell array to save patch data
            cc_faces_cell = cell(1,self.arm_info.n_links_and_joints) ;
            cc_verts_cell = cell(1,self.arm_info.n_links_and_joints) ;

            % create links as rectangles
            for l_idx = 1:self.arm_info.n_links_and_joints
                link = self.arm_info.links(l_idx).size ;

                switch self.arm_info.links(l_idx).shape
                    case 'cuboid'
                        [~,link_vertices] = make_cuboid_for_patch(link) ;
                    case 'ellipsoid'
                        link = link./2 ;
                        [~,link_vertices] = make_ellipsoid_for_patch(link(1),link(2),link(3),zeros(3,1),6) ;
                    case 'cylinder'
                        % l = (length, radius, (not used))
                        [~,link_vertices] = make_cylinder_for_patch(link(2)/2,link(1),10,true,true) ;
                        R = axang2rotm([0 1 0 pi/2]) ;
                        link_vertices = (R*link_vertices')' ;
                    otherwise
                        error('Invalid link type! Pick cuboid, ellipsoid, or cylinder!')
                end
                link_faces = convhull(link_vertices) ;

                % fill in cell array
                cc_faces_cell{l_idx} = link_faces ;
                cc_verts_cell{l_idx} = link_vertices ;
            end
            
            % fill in collision check data object
            self.collision_patch_data.faces = cc_faces_cell ;
            self.collision_patch_data.vertices = cc_verts_cell ;
        end
        
        % get collision check volume
        function out = getCollisionObject(self,options)
            arguments
                self
                options.q = self.arm_state.position(:,end)
                options.time = []
            end
            % out_volume = A.get_collision_check_volume(q)
            %
            % Given a configuration q...
            % If the agent is 2-D, return a polyline representing the
            % agent at that configuration. This can be used with polyxpoly
            % for collision checking.
            %
            % If the agent is 3-D, return a
            % patch structure with two fields (faces and vertices) that can
            % be used with SurfaceIntersection for collision checking.
            
            % get position given or the time to query.
            q = options.q;
            if ~isempty(options.time)
                state = self.arm_state.get_state(options.time);
                q = state.q;
            end
            
            [R,T] = self.kinematics.get_link_rotations_and_translations(q) ;
            
            F_cell = self.collision_patch_data.faces ;
            V_cell = self.collision_patch_data.vertices ;
            
            % Create the Patch3dObject object
            out = Patch3dObject;
            out.parent_uuid = self.arm_info.uuid;

            switch self.arm_info.dimension
                % case 2
                %     V = [] ;
                %     for idx = 1:length(F_cell)
                %         V_idx = R{idx}*V_cell{idx}' + T{idx} ;
                %         V = [V, nan(2,1), V_idx(:,F_cell{idx})] ;
                %     end
                    
                %     out = V(:,2:end) ;
                case 3
                    F = [] ;
                    V = [] ;
                    group_ends = [];
                    
                    N_verts = 0 ;
                    
                    for idx = 1:length(F_cell)
                        F_idx = F_cell{idx} + N_verts ;
                        V_idx = (R{idx}*V_cell{idx}' + T{idx})' ;
                        
                        F = [F ; F_idx] ;
                        V = [V ; V_idx] ;
                        
                        N_verts = size(V,1) ;
                        group_ends = [group_ends; N_verts];
                    end
                    
                    out.faces = F ;
                    out.vertices = V ;
                    out.group_ends = group_ends;
            end
        end
    end
end