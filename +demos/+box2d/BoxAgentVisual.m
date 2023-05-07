classdef BoxAgentVisual < rtd.sim.systems.patch_visual.PatchVisualObject & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle
    properties
        box_info demos.box2d.BoxAgentInfo = demos.box2d.BoxAgentInfo.empty()
        box_state rtd.entity.components.GenericEntityState = rtd.entity.components.GenericEntityState.empty()
        
        patch_style(1,1) struct
        plot_patch_data(1,1) struct
        
        set_view_when_animating(1,1) logical
        animation_view(1,1) int8
        
        plot_data
    end
    
    methods (Static)
        function options = defaultoptions()
            options.face_color = [1 0 1];
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
        function self = BoxAgentVisual(box_info,box_state_component,optionsStruct,options)
            arguments
                box_info demos.box2d.BoxAgentInfo
                box_state_component rtd.entity.components.GenericEntityState
                optionsStruct struct = struct()
                options.face_opacity
                options.edge_color
                options.edge_opacity
                options.edge_width
                options.set_view_when_animating
                options.animation_view
                options.verboseLevel
                options.name
            end
            options.face_color = box_info.color;
            self.mergeoptions(optionsStruct, options);
            
            self.box_info = box_info;
            self.box_state = box_state_component;
            
            self.reset();
        end
        
        function reset(self, optionsStruct, options)
            arguments
                self
                optionsStruct struct = struct()
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
            side_len = [self.box_info.width, self.box_info.height];
            center = zeros(dim,1) ;
            
            [F,V] = rtd.functional.geometry.make_box(side_len,center) ;
            
            self.plot_patch_data.faces = F ;
            self.plot_patch_data.vertices = V ;
        end
    end
    
    % Plotting
    methods
        function plot(self, options)
            arguments
                self
                options.time(1,1) double = self.box_state.time(end)
            end
            
            % Get the position         
            self.box_state.get_state(0)
            state = self.box_state.get_state(options.time);
            
            % Shift the patch points
            V = self.plot_patch_data.vertices + state.state.';
            F = self.plot_patch_data.faces;
            
            % Plot/update them
            if self.isPlotDataValid('body')
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
            self.plot(time=t);
        end
    end
end