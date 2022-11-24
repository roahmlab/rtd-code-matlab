classdef CollisionComponent < handle
    %CollisionComponent
    % TODO Complete
    
    properties
        % General information of the robot arm
        arm_info RoboticsToolboxArmRobotInfo
        
        arm_state_component
        
        patch_component
        
        collision_check_patch_data
    end
    
    methods
        function self = CollisionComponent(arm_info,arm_state_component,patch_component)
            self.arm_info = arm_info;
            self.arm_state_component = arm_state_component;
            self.patch_component = patch_component;
        end
        %% get collision check volume
        function out = get_collision_check_volume(A,q)
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
            
            if nargin < 2
                q = A.state(A.joint_state_indices,1) ;
            end
            
            [R,T] = A.get_link_rotations_and_translations(q) ;
            
            F_cell = A.collision_check_patch_data.faces ;
            V_cell = A.collision_check_patch_data.vertices ;
            
            switch A.dimension
                case 2
                    V = [] ;
                    for idx = 1:length(F_cell)
                        V_idx = R{idx}*V_cell{idx}' + T{idx} ;
                        V = [V, nan(2,1), V_idx(:,F_cell{idx})] ;
                    end
                    
                    out = V(:,2:end) ;
                case 3
                    F = [] ;
                    V = [] ;
                    
                    N_verts = 0 ;
                    
                    for idx = 1:length(F_cell)
                        F_idx = F_cell{idx} + N_verts ;
                        V_idx = (R{idx}*V_cell{idx}' + T{idx})' ;
                        
                        F = [F ; F_idx] ;
                        V = [V ; V_idx] ;
                        
                        N_verts = size(V,1) ;
                    end
                    
                    out.faces = F ;
                    out.vertices = V ;
            end
        end
        
        %% collision check data
        function create_collision_check_patch_data(A)
            % A.create_collision_check_patch_data()
            %
            % Create a cell array of faces vectors (each 1-by-NF) and a
            % cell array of vertices arrays (each NV-by-2) to be used for
            % collision checking.
            
            
            switch A.dimension
                case 2
                    cc_faces_cell = A.link_plot_data.link_faces ;
                    cc_verts_cell = A.link_plot_data.link_vertices ;
                case 3
                    % set up cell array to save patch data
                    cc_faces_cell = cell(1,A.n_links_and_joints) ;
                    cc_verts_cell = cell(1,A.n_links_and_joints) ;
                    
                    % create links as rectangles
                    L = A.link_sizes ;
                    
                    for l_idx = 1:size(L,2)
                        l = L(:,l_idx) ;
                        
                        switch A.link_shapes{l_idx}
                            case 'cuboid'
                                [~,link_vertices] = make_cuboid_for_patch(l) ;
                            case 'ellipsoid'
                                l = l./2 ;
                                [~,link_vertices] = make_ellipsoid_for_patch(l(1),l(2),l(3),zeros(3,1),6) ;
                            case 'cylinder'
                                % l = (length, radius, (not used))
                                [~,link_vertices] = make_cylinder_for_patch(l(2)/2,l(1),10,true,true) ;
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
            end
            
            % fill in collision check data object
            A.collision_check_patch_data.faces = cc_faces_cell ;
            A.collision_check_patch_data.vertices = cc_verts_cell ;
        end
    end    
end

