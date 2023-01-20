classdef BoxPatchVisual < PatchVisualObject & rtd.mixins.NamedClass & rtd.mixins.Options & handle
    
    
    % Leftover Old Dependencies
    % make_cuboid_for_patch
    % make_box
    % check_if_plot_is_available - This is probably a major hog!
    
    properties
        box_info BoxObstacleInfo = BoxObstacleInfo.empty()
        box_state GenericStateComponent = GenericStateComponent.empty()
        
        patch_style(1,1) struct
        plot_patch_data(1,1) struct
        
        set_view_when_animating(1,1) logical
        animation_view(1,1) int8
        
        plot_data
    end
    
    methods (Static)
        function options = defaultoptions()
            options.face_color = [1 0 0];
            options.face_opacity = 0.2;
            options.edge_color = [0 0 0];
            options.edge_opacity = 0.75;
            options.edge_width = 1;
            options.set_view_when_animating = false;
            options.animation_view = 3;
            options.verboseLevel = 'INFO';
            options.name = '';
        end
    end
    
    methods
        function self = BoxPatchVisual(box_info,box_state_component,optionsStruct,options)
            arguments
                box_info BoxObstacleInfo
                box_state_component GenericStateComponent
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
            
            self.box_info = box_info;
            self.box_state = box_state_component;
            
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
            
            self.create_plot_patch_data();
            
            self.plot_data.body = [];
        end
    end
    
    % Geometry Generation
    methods
        function create_plot_patch_data(self)
            dim = self.box_info.dimension;
            side_len = self.box_info.side_lengths ;
            center = zeros(dim,1) ;
            
            switch dim
                case 2
                    [F,V] = make_box(side_len,center) ;
                case 3
                    [F,V] = make_cuboid_for_patch(side_len(1),side_len(2),side_len(3),center) ;
            end
            
            self.plot_patch_data.faces = F ;
            self.plot_patch_data.vertices = V ;
        end
    end
    
    % Plotting
    methods
        function plot(self, options)
            arguments
                self BoxPatchVisual
                options.time(1,1) double = self.box_state.time(end)
            end
            
            % Get the position
            state = self.box_state.get_state(options.time);
            
            % Shift the patch points
            V = self.plot_patch_data.vertices + state.state.';
            F = self.plot_patch_data.faces;
            
            % Plot/update them
            if check_if_plot_is_available(self,'body')
                self.plot_data.body.Faces = F ;
                self.plot_data.body.Vertices = V ;
            else
                patch_data = patch('Faces',F,...
                                   'Vertices',V,...
                                   self.patch_style) ;
                self.plot_data.body = patch_data ;
            end
        end
        
        % Compat
        function plot_at_time(self,t)
            self.plot(time=t) ;
        end
    end
end