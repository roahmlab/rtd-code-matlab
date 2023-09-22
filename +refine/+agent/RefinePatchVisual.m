classdef RefinePatchVisual < rtd.sim.systems.patch_visual.PatchVisualObject & rtd.util.mixins.NamedClass & rtd.util.mixins.Options & handle
    properties
        position_indices
        heading_index

        footprint_vertices_for_plotting
        plot_data
        state
        time
        
    end

%     properties
%         plot_data (1,1) struct
%     end

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

        function A = RefinePatchVisual()
%             A.plot_data = struct('footprint',struct,'arrow',struct,'trajectory',struct,'pretty_footprint',[],'wheel_plot_data',[]);
             
        end
% 
%         function getPlotData(A)
% %             plot_data = struct(); % Initialize the plot_data structure
%             
%             % Example: Store or generate footprint plot data
%             plot_data.footprint.Vertices = []; % Define the vertices of the footprint
%             plot_data.footprint.FaceColor = 'red'; % Define the face color
%             plot_data.footprint.EdgeColor = 'blue'; % Define the edge color
%             
%             % Example: Store or generate arrow plot data
%             plot_data.arrow.Vertices = 1; % Define the vertices of the arrow
%             plot_data.arrow.FaceColor = 'red'; % Define the face color
%             plot_data.arrow.EdgeColor = 'blue'; % Define the edge color
%             
%             % Example: Store or generate trajectory plot data
%             plot_data.trajectory.XData = 0; % Define the X data for the trajectory
%             plot_data.trajectory.YData = 0; % Define the Y data for the trajectory
%             
%             % ... Add other plot elements as needed
%             plot_data.pretty_footprint.Vertices = 0;
%             plot_data.wheel_plot_data = struct();
%             plot_data.wheel_plot_data.XData = 0;
%             plot_data.wheel_plot_data.YData = 0;
%     
%             % Return the completed plot_data structure
% %             plot_data_ = plot_data;
% end

        function plot(A)
            if ~isempty(A.time)
                A.plot_at_time(A.time(end));
                A.plot_wheel_at_time(A.time(end));
            else
                A.plot_at_time(A.time());
                A.plot_wheel_at_time(A.time());
            end
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
            
%             self.create_baselink_plot_patch_data();
%             self.create_link_plot_patch_data();
            
            self.plot_data.baselink = [];
            self.plot_data.links = [];
        end

        function plot_at_time(A,t)
            % compute footprint for plot
            z_t = match_trajectories(t,A.time,A.state);
            p_t = z_t(A.position_indices);
            h_t = z_t(A.heading_index);
            R_t = rotation_matrix_2D(h_t);
            fp_t = A.footprint_vertices(:,1:end-1);
            N_fp = size(fp_t,2);
            V_fp = R_t*fp_t + repmat(p_t,1,N_fp);
            
            % make arrow for plot
            V_arrow = R_t*A.arrow_vertices + repmat(p_t,1,3);
            
            % plot
            hold_check = hold_switch();
            
            if check_if_plot_is_available(A,'footprint')
                A.plot_data.footprint.Vertices = V_fp';
                A.plot_data.arrow.Vertices = V_arrow' ;
            else
                % plot footprint
                fp_data = patch(V_fp(1,:),V_fp(2,:),A.plot_footprint_color,...
                    'EdgeColor',A.plot_footprint_edge_color,...
                    'FaceAlpha',A.plot_footprint_opacity,...
                    'EdgeAlpha',A.plot_footprint_edge_opacity) ;
                
                % plot arrow on footprint
                arrow_data = patch(V_arrow(1,:),V_arrow(2,:),A.plot_arrow_color,...
                    'EdgeColor',A.plot_arrow_color,...
                    'FaceAlpha',A.plot_arrow_opacity,...
                    'EdgeAlpha',A.plot_arrow_opacity) ;
                
                % save plot data
                A.plot_data.footprint = fp_data ;
                A.plot_data.arrow = arrow_data ;
            end
            
            if A.plot_trajectory_at_time_flag
                % get the executed path up to the current time
                X = A.state(A.position_indices,:) ;
                T_log = A.time <= t ;
                X = X(:,T_log) ;
                
                % plot it
                if check_if_plot_is_available(A,'trajectory')
                    A.plot_data.trajectory.XData = X(1,:) ;
                    A.plot_data.trajectory.YData = X(2,:) ;
                end
                    traj_data = plot_path(X,'k-','LineWidth',3) ;
                    A.plot_data.trajectory = traj_data ;
            end
            
            hold_switch(hold_check) ;
            A.plot_wheel_at_time(t);
        end
        
        function plot_wheel_at_time(A,t)
            wheel_size = [0.7 0.4];
            wheel = make_box(wheel_size);
            wheel_position = [-2   -2  1.5 1.5
                              -0.75 0.75 -0.75 0.75];
            wheel_vertices = [];
            for i = 1:4
                wheel_vertices = [wheel_vertices wheel+repmat(wheel_position(:,i),[1,5]) [NaN;NaN]];
            end
            % compute footprint for plot
            z_t = match_trajectories(t,A.time,A.state);
            p_t = z_t(A.position_indices);
            h_t = z_t(A.heading_index);
            delta_t = z_t(6);
            R_r = rotation_matrix_2D(h_t); 
            V_ft = R_r*A.footprint_vertices_for_plotting + repmat(p_t,1,size(A.footprint_vertices_for_plotting,2));
            R_f = rotation_matrix_2D(h_t+delta_t);
            V_all = R_r*wheel_vertices + repmat(p_t,1,size(wheel_vertices,2)) ;
            
            for i = 1:4
                if i == 3 || i == 4
                    wheel_vert = V_all(:,6*i-5:6*i-1);
                    wheel_center = repmat( 0.5*(max(wheel_vert,[],2)+min(wheel_vert,[],2)),[1,5]);
                    origion_vert = wheel_vert - wheel_center;
                    V_all(:,6*i-5:6*i-1) = R_f * origion_vert + wheel_center;
                end               
            end
            
            if check_if_plot_is_available(A,'pretty_footprint')
                A.plot_data.pretty_footprint.Vertices = V_ft' ;
                uistack(A.plot_data.pretty_footprint, 'top')
            else
                % plot footprint
                fp_data = patch(V_ft(1,:),V_ft(2,:),A.plot_footprint_color,...
                    'EdgeColor',A.plot_footprint_edge_color,...
                    'FaceAlpha',A.plot_footprint_opacity,...
                    'EdgeAlpha',A.plot_footprint_edge_opacity) ;
                
                
                % save plot data
                A.plot_data.pretty_footprint = fp_data ;
            end
            if check_if_plot_is_available(A,'wheel_plot_data')
                for i = 1:4
                    A.plot_data.wheel_plot_data{i}.XData =  V_all(1,6*i-5:6*i-1) ;
                    A.plot_data.wheel_plot_data{i}.YData =  V_all(2,6*i-5:6*i-1);
                    uistack(A.plot_data.wheel_plot_data{i}, 'top')
                end
                
            else
                for i =1:4
                    h = fill( V_all(1,6*i-5:6*i-1), V_all(2,6*i-5:6*i-1),A.wheel_color) ;
                    A.plot_data.wheel_plot_data{i} = h ;
                    h.FaceAlpha = A.plot_footprint_opacity;
                    h.EdgeAlpha = A.plot_footprint_edge_opacity;
                end
            end
        end

        function make_footprint_plot_data(A)
            switch length(A.footprint)
                case 1
                    A.footprint_vertices = make_circle(A.footprint) ;
                case 2
                    A.footprint_vertices = make_box(A.footprint) ;
            end
        end
        
        function make_arrow_plot_data(A)
            % make arrow for plot
            t_arrow = [0, 2*pi/3, 4*pi/3] ;
            
            switch length(A.footprint)
                case 1
                    % btw, this is 0.4x because the footprint is a radius
                    x_arrow = 0.4*A.footprint*cos(t_arrow) ;
                    y_arrow = 0.25*A.footprint*sin(t_arrow) ;
                case 2
                    x_arrow = 0.2*A.footprint(1)*cos(t_arrow) ;
                    y_arrow = 0.1*A.footprint(2)*sin(t_arrow) ;
            end
            A.arrow_vertices = [x_arrow ; y_arrow];
        end

        function animate(A,save_gif,time_interval)
        % method: animate(save_gif)
        %
        % Given the agent's executed trajectory, animate it for the
        % duration given by A.time. The time between animated frames is
        % given by A.animation_time_discretization.

            if nargin < 2
                save_gif = false ;
                start_gif = false ;
            else
                start_gif = true ;
                filename = A.gif_setup() ;
            end
            
            if nargin < 3
                time_interval = [A.time(1), A.time(end)] ;
            end

            % get timing info
            t_vec = time_interval(1):A.animation_time_discretization:time_interval(end) ;
            frame_rate = A.animation_time_discretization / A.animation_playback_rate ;

            % get axis limits
            lims = A.get_axis_lims() ;

            for t_idx = t_vec
                % create plot
                A.plot_at_time(t_idx)

                if A.animation_set_axes_flag
                    axis equal
                    axis(lims)
                end
                
                if A.animation_set_view_flag
                    view(A.animation_view)
                end

                % create gif
                if save_gif
                    % get current figure
                    fh = get(groot,'CurrentFigure') ;
                    frame = getframe(fh) ;
                    im = frame2im(frame);
                    [imind,cm] = rgb2ind(im,256);

                    if start_gif
                        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,...
                                'DelayTime',frame_rate) ;
                        start_gif = false ;
                    else 
                        imwrite(imind,cm,filename,'gif','WriteMode','append',...
                                'DelayTime',frame_rate) ;
                    end
                else
                    pause(frame_rate)
                end
            end
        end
        
        function lims = get_axis_lims(A)
            z = A.state(A.position_indices,:) ;
            xmin = min(z(1,:)) - A.animation_plot_buffer ;
            xmax = max(z(1,:)) + A.animation_plot_buffer ;
            ymin = min(z(2,:)) - A.animation_plot_buffer ;
            ymax = max(z(2,:)) + A.animation_plot_buffer ;
            lims = [xmin xmax ymin ymax] ;
        end
    
        function filename = gif_setup(A)       
            filename = A.animation_gif_filename ;

            dir_content = dir(pwd) ;
            filenames   = {dir_content.name} ;
            file_check  = any(cellfun(@(x) strcmp(filename,x),filenames)) ;
            filename_new = filename ;
            cur_int = 1 ;

            while file_check
                filename_new = [filename(1:end-4),'_',num2str(cur_int),filename(end-3:end)] ;
                file_check  = any(cellfun(@(x) strcmp(filename_new,x),filenames)) ;
                cur_int = cur_int + 1 ;
            end

            filename = filename_new ;
        end

        

    end

end
