classdef ArmourCadPatchVisual < armour.agent.ArmourPatchVisual
    % Extended form the ArmourPatchVisual component to plot CAD models
    % instead. Lighting should NOT run in this.
    
    methods (Static)
        function options = defaultoptions()
            options = armour.agent.ArmourPatchVisual.defaultoptions();
            options.face_color = [0.8 0.8 1];
            options.face_opacity = 1;
            options.edge_color = [0 0 1];
            options.edge_opacity = 0;
            options.edge_width = 1.25;
        end
    end
    
    % Geometry Generation
    methods
        function create_link_plot_patch_data(self)
            self.vdisp('Loading CAD files for arm plotting!', 'INFO')
            for i = 1:self.arm_info.robot.NumBodies
                stl_file = self.arm_info.robot.Bodies{i}.Visuals{1};
                stl_file = extractAfter(stl_file, 'Mesh:');
                mesh{i, 1} = stlread(stl_file);
            end
            
            
            % check to make sure the CAD data are patches, not triangulations
            triangulated_flag = false ;
            for idx = 1:length(mesh)
                current_data = mesh{idx} ;
                if isa(current_data,'triangulation')
                    triangulated_flag = true ;
                    new_data.faces = current_data.ConnectivityList ;
                    new_data.vertices = current_data.Points ;
                    mesh{idx} = new_data ;
                end
            end
            
            if triangulated_flag
                self.vdisp('STL read returned a triangulated data format, but we fixed it :)', 'WARN')
            end
            
            % Flatten to a compatible format with the prior plot routine
            mesh = [mesh{:}];
            self.link_plot_data.link_faces = {mesh.faces};
            self.link_plot_data.link_vertices = {mesh.vertices};
        end
    end
end