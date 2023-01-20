classdef ArmourPatchVisual < rtd.core.systems.patch_visual.PatchVisualObject & rtd.core.mixins.NamedClass & rtd.core.mixins.Options & handle
    
    
    % Leftover Old Dependencies
    % make_cuboid_for_patch
    % make_box
    % make_oval
    % make_ellipsoid_for_patch
    % make_cylinder_for_patch
    % check_if_plot_is_available - This is probably a major hog!
    
    properties
        arm_info ArmourAgentInfo = ArmourAgentInfo.empty()
        arm_state ArmourAgentState = ArmourAgentState.empty()
        kinematics ArmKinematics = ArmKinematics.empty()
        
        patch_style(1,1) struct
        link_plot_data(1,1) struct
        
        set_view_when_animating(1,1) logical
        animation_view(1,1) int8
        
        plot_data = struct
    end
    
    methods (Static)
        function options = defaultoptions()
            options.face_color = [0 0 1];
            options.face_opacity = 0.1;
            options.edge_color = [0 0 1];
            options.edge_opacity = 1.0;
            options.edge_width = 1.25;
            options.set_view_when_animating = true;
            options.animation_view = 3;
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end
    
    methods
        function self = ArmourPatchVisual(arm_info,arm_state_component,kinematics_component,optionsStruct,options)
            arguments
                arm_info ArmourAgentInfo
                arm_state_component ArmourAgentState
                kinematics_component ArmKinematics
                optionsStruct struct = struct()
                options.face_color
                options.face_opacity
                options.edge_color
                options.edge_opacity
                options.edge_width
                options.set_view_when_animating
                options.animation_view
                options.verboseLevel
                options.name
            end
            self.mergeoptions(optionsStruct, options);
            
            self.arm_info = arm_info;
            self.arm_state = arm_state_component;
            self.kinematics = kinematics_component;
            
            %self.reset();
        end
        
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
                options.face_color
                options.face_opacity
                options.edge_color
                options.edge_opacity
                options.edge_width
                options.set_view_when_animating
                options.animation_view
                options.verboseLevel
                options.name
            end
            options = self.mergeoptions(optionsStruct, options);
            
            % Set up verbose output
            self.name = options.name;
            self.set_vdisplevel(options.verboseLevel);
            
            self.patch_style.FaceColor = options.face_color;
            self.patch_style.FaceAlpha = options.face_opacity;
            self.patch_style.EdgeColor = options.edge_color;
            self.patch_style.EdgeAlpha = options.edge_opacity;
            self.patch_style.LineWidth = options.edge_width;
            self.set_view_when_animating = options.set_view_when_animating;
            self.animation_view = options.animation_view;
            
            self.create_baselink_plot_patch_data();
            self.create_link_plot_patch_data();
            
            self.plot_data.baselink = [];
            self.plot_data.links = [];
        end
    end
    
    % Geometry Generation
    methods
        function create_baselink_plot_patch_data(self)
            % create baselink cone for plotting
            [faces,vertices] = make_cuboid_for_patch(0.025, 0.025, 0.025, [0;0;0]);
            
            % save
            self.link_plot_data.baselink_faces = faces;
            self.link_plot_data.baselink_vertices = vertices;
        end
        
        function create_link_plot_patch_data(self)
            % A.create_link_plot_patch_data()
            %
            % This method fills in self.link_plot_data with a cell array of
            % faces vectors (each 1-by-NF) and a cell array of vertices
            % arrays (each NV-by-2).
            
            % set up cell array to save patch data
            plot_faces_cell = cell(1,self.arm_info.n_links_and_joints);
            plot_verts_cell = cell(1,self.arm_info.n_links_and_joints);
                    
            % create links
            for l_idx = 1:self.arm_info.n_links_and_joints
                link = self.arm_info.links(l_idx).size ;
                
                % create link based on link type
                switch self.arm_info.links(l_idx).shape
                    case 'box'
                        [faces, vertices] = make_box(link) ;
                    case 'oval'
                        [faces, vertices] = make_oval(link) ;
                    case 'cuboid'
                        % create box for link that is slightly shorter than
                        % the actual volume of the box for prettiness
                        % purposes
                        % l(1) = l(1) - 0.045 ;
                        [faces,vertices] = make_cuboid_for_patch(link) ;
                    case 'ellipsoid'
                        % remember that the link sizes specify the
                        % diameters of the link in each dimension
                        link = link./2 ;
                        [faces,vertices] = make_ellipsoid_for_patch(link(1),link(2),link(3),zeros(3,1),6) ;
                    case 'cylinder'
                        % l = (length, radius, (not used))
                        [faces,vertices] = make_cylinder_for_patch(link(2)/2,link(1),10,true,true) ;
                        R = axang2rotm([0 1 0 pi/2]) ;
                        vertices = (R*vertices')' ;
                    otherwise
                        error('Invalid link type! Pick cuboid, ellipsoid, or cylinder.')
                end

                % fill in cell array
                plot_faces_cell{l_idx} = faces ;
                plot_verts_cell{l_idx} = vertices ;
            end
            
            % fill in plot link data object
            self.link_plot_data.link_faces = plot_faces_cell;
            self.link_plot_data.link_vertices = plot_verts_cell;
        end
    end
    
    % Plotting
    methods
        function plot_at_time(self,t)
            self.plot(time=t) ;
        end
        
        function plot(self,options)
            arguments
                self
                options.time(1,1) double = self.arm_state.time(end)
            end
            
            self.plot_baselink() ;
            self.plot_links(options.time) ;
        end
        
        function plot_baselink(self)
            % plot baselink
            BF = self.link_plot_data.baselink_faces ;
            BV = self.link_plot_data.baselink_vertices ;
            
            if check_if_plot_is_available(self,'baselink')
                self.plot_data.baselink.Faces = BF ;
                self.plot_data.baselink.Vertices = BV ;
            else
                baselink_data = patch('Faces',BF,'Vertices',BV,self.patch_style) ;
                self.plot_data.baselink = baselink_data ;
            end
        end
        
        function plot_links(self,time_or_config)
            % TEMP FIX FOR CADPATCHVISUAL
            cad_flag = strcmp(self.classname, 'ArmourCadPatchVisual');
            % get the rotations and translations at the current time
            [R,T] = self.kinematics.get_link_rotations_and_translations(time_or_config, cad_flag);
            
            % generate plot data for each link
            % cellfun is easier to see the math from
            link_verts = cellfun( ...
                @(R, vert, T){(R * vert.' + T).'}, ...
                R, self.link_plot_data.link_vertices, T);
            
            if check_if_plot_is_available(self,'links')
                for idx = 1:self.arm_info.n_links_and_joints
                    self.plot_data.links(idx).Faces = self.link_plot_data.link_faces{idx} ;
                    self.plot_data.links(idx).Vertices = link_verts{idx} ;
                end
            else
                link_array = gobjects(1, self.arm_info.n_links_and_joints);
                for idx = 1:self.arm_info.n_links_and_joints
                    link_array(idx) = patch(...
                        'Faces',    self.link_plot_data.link_faces{idx},...
                        'Vertices', link_verts{idx},...
                         self.patch_style);
                end
                self.plot_data.links = link_array ;
            end
        end
        
        function lims = get_axis_lims(self)
            lims = self.arm_info.reach_limits;
        end
    end
end