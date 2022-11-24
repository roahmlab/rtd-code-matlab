classdef PatchDataComponent < handle
    %PatchDataComponent
    % TODO migrate out
    
    properties
        % General information of the robot arm
        arm_info RoboticsToolboxArmRobotInfo
        
        link_plot_data
        link_CAD_data
    end
    
    methods
        function self = PatchDataComponent(arm_info)
            self.arm_info = arm_info;
            self.create_plot_patch_data()
        end
        
        % ARMTD
%         function [faces,vertices] = create_baselink_plot_patch_data(A)
%             switch A.dimension
%                 case 2
% %                     % create baselink triangle for plotting
% %                     vertices = 0.05.*[-1, 1, 0 ;
% %                         0, 0, 1 ]' ;
% %                     faces = [1 2 3 1] ;
%                     % create tiny square for plotting
%                     vertices = 0.025.*[-1, 1, 1, -1 ;
%                         -1, -1, 1, 1]' ;
%                     faces = [1 2 3 4 1] ;
%                     
%                 case 3
%                     % create baselink cone for plotting
%                     [faces,vertices] = make_cone_for_patch(0.05,0.05) ;
%             end
%         end
        
        % ARMOUR -> move to visual only
        function [faces,vertices] = create_baselink_plot_patch_data(self)
            % create baselink cone for plotting
            [faces,vertices] = make_cuboid_for_patch(0.025, 0.025, 0.025, [0;0;0]) ;
        end
        
        % MIGRATE TO ARM ROBOT INFO
        function create_plot_patch_data(self)
            % A.create_plot_patch_data()
            %
            % This method fills in A.link_plot_data with a cell array of
            % faces vectors (each 1-by-NF) and a cell array of vertices
            % arrays (each NV-by-2).
            
            % set up cell array to save patch data
            plot_faces_cell = cell(1,self.arm_info.n_links_and_joints) ;
            plot_verts_cell = cell(1,self.arm_info.n_links_and_joints) ;
            
            % get link sizes
            L = self.arm_info.link_sizes ;
            
            % make baselink plot data
            [baselink_faces,baselink_vertices] = self.create_baselink_plot_patch_data() ;
                    
            % create links
            for l_idx = 1:size(L,2)
                l = L(:,l_idx) ;
                
                % create link based on link type
                switch self.arm_info.link_shapes{l_idx}
                    case 'box'
                        [link_faces, link_vertices] = make_box(l) ;
                    case 'oval'
                        [link_faces, link_vertices] = make_oval(l) ;
                    case 'cuboid'
                        % create box for link that is slightly shorter than
                        % the actual volume of the box for prettiness
                        % purposes
                        % l(1) = l(1) - 0.045 ;
                        [link_faces,link_vertices] = make_cuboid_for_patch(l) ;
                    case 'ellipsoid'
                        % remember that the link sizes specify the
                        % diameters of the link in each dimension
                        l = l./2 ;
                        [link_faces,link_vertices] = make_ellipsoid_for_patch(l(1),l(2),l(3),zeros(3,1),6) ;
                    case 'cylinder'
                        % l = (length, radius, (not used))
                        [link_faces,link_vertices] = make_cylinder_for_patch(l(2)/2,l(1),10,true,true) ;
                        R = axang2rotm([0 1 0 pi/2]) ;
                        link_vertices = (R*link_vertices')' ;
                    otherwise
                        error('Invalid link type! Pick cuboid, ellipsoid, or cylinder.')
                end

                % fill in cell array
                plot_faces_cell{l_idx} = link_faces ;
                plot_verts_cell{l_idx} = link_vertices ;
            end
            
            % fill in plot link data object
            self.link_plot_data.link_faces = plot_faces_cell ;
            self.link_plot_data.link_vertices = plot_verts_cell ;
            self.link_plot_data.baselink_faces = baselink_faces ;
            self.link_plot_data.baselink_vertices = baselink_vertices ;
        end
        
        % MIGRATE TO ARM ROBOT INFO
        function load_CAD_arm_patch_data(self, temp_link_CAD_data)
            %self.vdisp('Loading CAD files for arm plotting!',1)
            
            % check to make sure the CAD data are patches, not triangulations
            triangulated_flag = false ;
            for idx = 1:length(temp_link_CAD_data)
                current_data = temp_link_CAD_data{idx} ;
                if isa(current_data,'triangulation')
                    triangulated_flag = true ;
                    new_data.faces = current_data.ConnectivityList ;
                    new_data.vertices = current_data.Points ;
                    temp_link_CAD_data{idx} = new_data ;
                end
            end
            
            if triangulated_flag
                %self.vdisp('STL read returned a triangulated data format, but we fixed it :)',7)
            end
            
            self.link_CAD_data = temp_link_CAD_data ;
        end
    end    
end

