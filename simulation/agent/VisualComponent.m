classdef VisualComponent < handle
    %VisualComponent
    % TODO Add from base agent & clean up / flesh out (animate, gif)
    
    properties
        % General information of the robot arm
        arm_info RoboticsToolboxArmRobotInfo
        
        arm_state_component
        
        patch_component
        
        plot_data
        
        edge_color
        edge_width
        edge_opacity
        face_color
        face_opacity
        
        use_cad
    end
    
    methods
        function self = VisualComponent(arm_info,arm_state_component,patch_component,use_cad)
            self.arm_info = arm_info;
            self.arm_state_component = arm_state_component;
            self.patch_component = patch_component;
            self.use_cad = use_cad;
        end
        
        function plot(self,~)
            self.plot_at_time(self.arm_state_component.time(end)) ;
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
            BF = self.patch_component.link_plot_data.baselink_faces ;
            BV = self.patch_component.link_plot_data.baselink_vertices ;
            
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
        
        function plot_links(self,time_or_config)
            % get the rotations and translations at the current time
            [R,T] = self.arm_state_component.get_link_rotations_and_translations(time_or_config) ;
            
            % generate plot data for each link
            link_verts = cell(1,self.arm_info.n_links_and_joints) ;
            for idx = 1:self.arm_info.n_links_and_joints
                link_verts{idx} = (R{idx}*self.patch_component.link_plot_data.link_vertices{idx}' + ...
                    T{idx})' ;
            end
            
            if check_if_plot_is_available(self,'links')
                for idx = 1:self.arm_info.n_links_and_joints
                    self.plot_data.links(idx).Faces = self.patch_component.link_plot_data.link_faces{idx} ;
                    self.plot_data.links(idx).Vertices = link_verts{idx} ;
                end
            else
                link_array = [] ;
                for idx = 1:self.arm_info.n_links_and_joints
                    link_data = patch('Faces',self.patch_component.link_plot_data.link_faces{idx},...
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
        
        function lims = get_axis_lims(A)
            % figure out the maximum length of the arm
            L = sum(A.link_sizes(1,:)) ;
            
            % create axis limits
            switch A.dimension
                case 2
                    lims = [-L,L,-L,L] ;
                case 3
%                     switch A.floor_normal_axis
%                         case 1
%                             lims = [-L, L, -L, L, -L, L] ;
%                         case 2
%                             lims = [-L, L, 0, L, -L, L] ;
%                         case 3
%                             lims = [-L,L,-L,L,0,L] ;
%                     end
                    lims = [-L, L, -L, L, -L, L] ;
                    
                    % in case base of robot is not at [0;0;0]:
                    lims = lims + [A.joint_locations(1, 1)*ones(1, 2),...
                        A.joint_locations(2, 1)*ones(1, 2),...
                        A.joint_locations(3, 1)*ones(1, 2)];
            end
        end
        %% ARMOUR
        function plot_links(A,time_or_config)
            if A.use_CAD_flag
                % get the rotations and translations at the current time
                if length(time_or_config) == 1
                    q = match_trajectories(time_or_config,A.time,A.state(A.joint_state_indices,:)) ;
                else
                    q = time_or_config ;
                end
                                
                % get transformations for the plot links
                cad_flag = true;
                [R,T] = A.get_link_rotations_and_translations(q,cad_flag) ;
                
                % set the number of plot links
                n = A.n_links_and_joints;
                
                % generate plot data for each link
                link_verts = cell(1,n) ;
                for idx = 1:n
                    link_verts{idx} = (R{idx}*A.link_CAD_data{idx}.vertices' + ...
                        T{idx})' ;
                end
                
                if check_if_plot_is_available(A,'links')
                    for idx = 1:n
                        A.plot_data.links(idx).Faces = A.link_CAD_data{idx}.faces ;
                        A.plot_data.links(idx).Vertices = link_verts{idx} ;
                    end
                else
                    link_array = [] ;
                    for idx = 1:n
                        link_data = patch('Faces',A.link_CAD_data{idx}.faces,...
                            'Vertices',link_verts{idx},...
                            'FaceColor',A.link_plot_face_color,...
                            'FaceAlpha',A.link_plot_face_opacity,...
                            'EdgeColor',A.link_plot_edge_color,...
                            'LineWidth',A.link_plot_edge_width,...
                            'EdgeAlpha',A.link_plot_edge_opacity) ;
                        link_array = [link_array, link_data] ;
                    end
                    A.plot_data.links = link_array ;
                    
                    % turn camlight on
                    camlight
                end
                % show(A.robot, q, 'PreservePlot', false);
                % drawnow;
            else
                plot_links@robot_arm_agent(A,time_or_config) ;
            end
        end
        
        %% custom axis limits
        function lims = get_axis_lims(A)
            % figure out the maximum length of the arm
            L = sum(A.link_sizes(1,:)) ;
            
            % create axis limits
            switch A.dimension
                case 2
                    lims = [-L,L,0,L] ;
                case 3
                    lims = [-0.5*L, L, -L, L, -L, 0.75*L] ;
                    
                    % in case base of robot is not at [0;0;0]:
                    lims = lims + [A.joint_locations(1, 1)*ones(1, 2), A.joint_locations(2, 1)*ones(1, 2), A.joint_locations(3, 1)*ones(1, 2)];
                    
                    % make z = 0 the ground
                    lims(5) = 0;
            end
        end
    end    
end

