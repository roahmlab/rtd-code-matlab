classdef VisualComponent < handle
    %VisualComponent
    % TODO Add from base agent & clean up / flesh out (animate, gif)
    
    properties
        % General information of the robot arm
        arm_info RoboticsToolboxArmRobotInfo
        
        arm_state
        
        kinematics
        
        plot_data
        link_plot_data
        link_CAD_data
        
        edge_color
        edge_width
        edge_opacity
        face_color
        face_opacity
        
        use_cad
        
        set_view_when_animating
        animation_view
    end
    
    methods
        function self = VisualComponent(arm_info,arm_state_component,kinematics_component,options)
            arguments
                arm_info
                arm_state_component
                kinematics_component
                options.use_cad = false;
                options.face_color = [0 0 1];
                options.face_opacity = 0.1;
                options.edge_color = [0 0 1];
                options.edge_opacity = 1.0;
                options.edge_width = 1.25;
                options.set_view_when_animating = true;
                options.animation_view = 3;
            end
            self.arm_info = arm_info;
            self.arm_state = arm_state_component;
            self.kinematics = kinematics_component;
            
            self.use_cad = options.use_cad;
            self.face_color = options.face_color;
            self.face_opacity = options.face_opacity;
            self.edge_color = options.edge_color;
            self.edge_width = options.edge_width;
            self.set_view_when_animating = options.set_view_when_animating;
            self.animation_view = options.animation_view;
        end
        
        % ARMOUR -> move to visual only
        function [faces,vertices] = create_baselink_plot_patch_data(self)
            % create baselink cone for plotting
            [faces,vertices] = make_cuboid_for_patch(0.025, 0.025, 0.025, [0;0;0]) ;
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
        
        function plot(self,~)
            self.plot_at_time(self.arm_state.time(end)) ;
        end
        
        function plot_at_time(self,t,~)
            if nargin < 2
                t = 0 ;
            end
            
            self.plot_baselink() ;
            self.plot_links(t) ;
        end
        
        function plot_baselink(self)
            % plot baselink
            BF = self.link_plot_data.baselink_faces ;
            BV = self.link_plot_data.baselink_vertices ;
            
            if check_if_plot_is_available(self,'baselink')
                self.plot_data.baselink.Faces = BF ;
                self.plot_data.baselink.Vertices = BV ;
            else
                baselink_data = patch('Faces',BF,'Vertices',BV,...
                    'FaceColor',self.face_color,...
                    'FaceAlpha',self.face_opacity,...
                    'EdgeColor',self.edge_color,...
                    'LineWidth',self.edge_width,...
                    'EdgeAlpha',self.edge_opacity) ;
                self.plot_data.baselink = baselink_data ;
            end
        end
        
        function plot_links_parent(self,time_or_config)
            % get the rotations and translations at the current time
            [R,T] = self.kinematics.get_link_rotations_and_translations(time_or_config) ;
            
            % generate plot data for each link
            link_verts = cell(1,self.arm_info.n_links_and_joints) ;
            for idx = 1:self.arm_info.n_links_and_joints
                link_verts{idx} = (R{idx}*self.link_plot_data.link_vertices{idx}' + ...
                    T{idx})' ;
            end
            
            if check_if_plot_is_available(self,'links')
                for idx = 1:self.arm_info.n_links_and_joints
                    self.plot_data.links(idx).Faces = self.link_plot_data.link_faces{idx} ;
                    self.plot_data.links(idx).Vertices = link_verts{idx} ;
                end
            else
                link_array = [] ;
                for idx = 1:self.arm_info.n_links_and_joints
                    link_data = patch('Faces',self.link_plot_data.link_faces{idx},...
                        'Vertices',link_verts{idx},...
                        'FaceColor',self.face_color,...
                        'FaceAlpha',self.face_opacity,...
                        'EdgeColor',self.edge_color,...
                        'LineWidth',self.edge_width,...
                        'EdgeAlpha',self.edge_opacity) ;
                    link_array = [link_array, link_data] ;
                end
                self.plot_data.links = link_array ;
            end
        end
%         
%         function lims = get_axis_lims(self)
%             % figure out the maximum length of the arm
%             L = sum(self.arm_info.link_sizes(1,:)) ;
%             
%             % create axis limits
%             switch self.arm_info.dimension
%                 case 2
%                     lims = [-L,L,-L,L] ;
%                 case 3
% %                     switch A.floor_normal_axis
% %                         case 1
% %                             lims = [-L, L, -L, L, -L, L] ;
% %                         case 2
% %                             lims = [-L, L, 0, L, -L, L] ;
% %                         case 3
% %                             lims = [-L,L,-L,L,0,L] ;
% %                     end
%                     lims = [-L, L, -L, L, -L, L] ;
%                     
%                     % in case base of robot is not at [0;0;0]:
%                     lims = lims + [self.arm_info.joint_locations(1, 1)*ones(1, 2),...
%                         self.arm_info.joint_locations(2, 1)*ones(1, 2),...
%                         self.arm_info.joint_locations(3, 1)*ones(1, 2)];
%             end
%         end
        %% ARMOUR
        function plot_links(self,time_or_config)
            if self.use_cad
                % get the rotations and translations at the current time
                if length(time_or_config) == 1
                    q = match_trajectories(time_or_config,self.arm_state.time,self.arm_state.position);
                else
                    q = time_or_config ;
                end
                                
                % get transformations for the plot links
                cad_flag = true;
                [R,T] = self.kinematics.get_link_rotations_and_translations(q,cad_flag) ;
                
                % set the number of plot links
                n = self.arm_info.n_links_and_joints;
                
                % generate plot data for each link
                link_verts = cell(1,n) ;
                for idx = 1:n
                    link_verts{idx} = (R{idx}*self.link_CAD_data{idx}.vertices' + ...
                        T{idx})' ;
                end
                
                if check_if_plot_is_available(self,'links')
                    for idx = 1:n
                        self.plot_data.links(idx).Faces = self.link_CAD_data{idx}.faces ;
                        self.plot_data.links(idx).Vertices = link_verts{idx} ;
                    end
                else
                    link_array = [] ;
                    for idx = 1:n
                        link_data = patch('Faces',self.link_CAD_data{idx}.faces,...
                            'Vertices',link_verts{idx},...
                            'FaceColor',self.face_color,...
                            'FaceAlpha',self.face_opacity,...
                            'EdgeColor',self.edge_color,...
                            'LineWidth',self.edge_width,...
                            'EdgeAlpha',self.edge_opacity) ;
                        link_array = [link_array, link_data] ;
                    end
                    self.plot_data.links = link_array ;
                    
                    % turn camlight on
                    camlight
                end
                % show(A.robot, q, 'PreservePlot', false);
                % drawnow;
            else
                self.plot_links_parent(time_or_config) ;
            end
        end
        
        %% custom axis limits
        function lims = get_axis_lims(self)
            % figure out the maximum length of the arm
            L = sum(self.arm_info.link_sizes(1,:)) ;
            
            % create axis limits
            switch self.arm_info.dimension
                case 2
                    lims = [-L,L,0,L] ;
                case 3
                    lims = [-0.5*L, L, -L, L, -L, 0.75*L] ;
                    
                    % in case base of robot is not at [0;0;0]:
                    lims = lims + [self.arm_info.joint_locations(1, 1)*ones(1, 2), self.arm_info.joint_locations(2, 1)*ones(1, 2), self.arm_info.joint_locations(3, 1)*ones(1, 2)];
                    
                    % make z = 0 the ground
                    lims(5) = 0;
            end
        end
    end    
end

