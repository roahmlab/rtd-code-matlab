classdef NewhighwayAgentHelper < agentHelper
    %% properties
    properties
        HLP;
        robotState
        worldState
        waypoint
        % partitions on u0
        u0_array        
%         t_move
        % reference data for plot
        ref_Z = [];
        proposed_ref_Z = [];
        t_real_start = [];
        t_proposed_start = [];
        
        prev_action = -1;
        cur_t0_idx = 1;
        saved_K = [];
        t_plan;    
        S 

        FRS_helper_handle = struct;
        truncating_factor;
        refinePlanner
        
        waypt_hist = [];
        K_hist = [];
        FRS_hist = {};
        mirror_hist = [];
        state_hist = [];
        type_manu_hist = [];
        time_hist = [];
        solve_time_hist = [];

    end
    %% methods
    methods
        function AH = NewhighwayAgentHelper(A, HLP,worldState, trajOptProps,varargin)
            %dummy for testing
            frs = 0;
            AH@agentHelper(A,frs,varargin{:}); %calls gen_paramter_standalone here
            AH.t_move =3;
            AH.HLP = HLP;
            AH.worldState = worldState;
            AH.truncating_factor = 1;
            refinePlanner = Refine_Planner(trajOptProps,AH);%no need to pass AH here, t_plan in trajOptProps.
            AH.refinePlanner = refinePlanner;

        end
        
        
        
        function [trajectory,tout] = gen_parameter_standalone(AH, worldinfo,agent_state,waypoints)
            AH.waypoint = waypoints;
            [trajectory,tout] = AH.refinePlanner.planTrajectory(agent_state(1:6),worldinfo,waypoints);%initially it was AH.robotstate,worldinfo,AH.worldState
           
        end
        
        function plot_selected_parameter_FRS(AH,K,type_manu,FRS,mirror_flag,agent_state,multiplier)
            if ~isempty(K)
                %clear data and then plot
                AH.FRS_helper_handle.XData = cell(3,1);
                AH.FRS_helper_handle.YData = cell(3,1);
                if type_manu == 1 % 1: speed change. 2: direction change. 3: lane change
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[K; 0],[0 0 1],2);
                else
                    AH.plot_zono_collide_sliced(FRS,mirror_flag,agent_state,[agent_state(4);K *multiplier],[0 1 0],2);
                end
            end
        end


        
        function [T, U, Z]=gen_ref(AH, K, reference_flag,agent_state, ref_time)
            % generate reference based on parameter and states
            if ~exist('agent_state','var')
                agent_info = AH.get_agent_info();
                agent_state = agent_info.state(:,end);
            end
            if ~exist('ref_time','var')
                ref_time = AH.A.time(end);
            end
            if ~exist('reference_flag','var')
                reference_flag = 1;
            end
            u_cur = agent_state(4) ;
            y_cur = agent_state(2) ;
            x_cur = agent_state(1) ;
            Au = K(1);
            Ay = K(2);
            t0_idx = K(3);
%             disp('t0 from h')
%             disp(t0_idx)
%             disp(AH.t_move)
            t0 = (t0_idx-1)*AH.t_move;
            type_manu = K(4);
            if type_manu == 3 % 1: speed change. 2: direction change. 3: lane change
                [T, U,Z] = gaussian_T_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            else
                [T, U,Z] = sin_one_hump_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            end
            
            if reference_flag
                AH.ref_Z=[AH.ref_Z;x_cur+Z(1,:);y_cur+Z(2,:)];% for plotting
                AH.t_real_start = [AH.t_real_start;ref_time];
            else
                AH.proposed_ref_Z=[AH.proposed_ref_Z;x_cur+Z(1,:);y_cur+Z(2,:)];% for plotting
                AH.t_proposed_start = [AH.t_proposed_start;ref_time];
            end
            
            
        end

        function reset(AH,flags,eps_seed)
            if ~exist('eps_seed','var')
                AH.A.reset();
            else
                rng(eps_seed)
                AH.A.reset();
            end
            AH.flags = flags;
            AH.ref_Z = [];
            AH.proposed_ref_Z = [];
            AH.t_real_start = [];
            AH.t_proposed_start = [];
            AH.K_hist = [];
            AH.waypt_hist = [];
            AH.FRS_hist = {};
            AH.mirror_hist = [];
            AH.state_hist = [];
            AH.type_manu_hist = [];
            AH.time_hist = [];
            if ~isempty(AH.HLP)
                AH.HLP.reset();
                if ~isempty(AH.HLP.plot_data.waypoints)
                    AH.HLP.plot_data.waypoints.XData = [];
                    AH.HLP.plot_data.waypoints.YData = [];
                end
            end
        end
        
        %% plot functions
        function plot(AH)
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on
            end
            if ~isempty(AH.planned_path)
               plot(AH.planned_path(1,:),AH.planned_path(2,:),'k-','LineWidth',1);
            end
            text(-250,15,"u="+num2str(AH.A.state(4,end))+"m/s",'Color','red','FontSize',15)
            
            if hold_check
                hold off ;
            end
            
        end
        function plot_zono_collide_sliced(AH,FRS,mirror_flag,agent_state,K,color,slice_level)
            if K(2) == 0
                slice_dim = 11;
                k_slice = K(1);
            else
                slice_dim = 12;
                k_slice = K(2);
            end

            
            for t_idx = 1:5:length(FRS.vehRS_save) 
                if slice_level == 0
                    zono_one = FRS.vehRS_save{t_idx};
                elseif slice_level == 1
                    zono_one = zonotope_slice(FRS.vehRS_save{t_idx}, [7;8;9], [agent_state(4);agent_state(5);agent_state(6)]);
                elseif slice_level == 2
                    zono_one = zonotope_slice(FRS.vehRS_save{t_idx}, [7;8;9;slice_dim], [agent_state(4);agent_state(5);agent_state(6);k_slice]);
                else
                    error('unknown slice_level in plot selected zono');
                end
                h = plot(zono_one,[1,2],'Color',color);
                if mirror_flag
                    h.YData = - h.YData;
                end
                XY = [h.XData(:) h.YData(:)];                                    
                theta = agent_state(3);
                R=[cos(theta) -sin(theta); sin(theta) cos(theta)];
                rotXY=XY*R'; %MULTIPLY VECTORS BY THE ROT MATRIX
                Xqr = reshape(rotXY(:,1), size(h.XData,1), []);
                Yqr = reshape(rotXY(:,2), size(h.YData,1), []);
                %SHIFTING
                h.XData = Xqr+agent_state(1);
                h.YData = Yqr+agent_state(2);
                
                AH.FRS_helper_handle.XData{slice_level+1} = [h.YData nan AH.FRS_helper_handle.XData{slice_level+1}];
                AH.FRS_helper_handle.YData{slice_level+1} = [h.XData nan AH.FRS_helper_handle.YData{slice_level+1}];
            end
            

        end

    end
end

%% helper function to generate obstacle structure for c++
function obj_mex = get_obs_mex(dyn_obs, bounds)
    all_pts = dyn_obs{1};
    all_vels = dyn_obs{2};
    obj_mex = [];
    n_obs = length(all_vels);
    for dyn_obs_idx = 1:n_obs
        dyn_obs_pts_start_idx = ((dyn_obs_idx-1) * 6) + 1;
        curr_pts = all_pts(:, ...
            dyn_obs_pts_start_idx:dyn_obs_pts_start_idx+3);
        deltas = max(curr_pts,[],2) - min(curr_pts,[],2);
        means = mean(curr_pts, 2);
        dyn_c_x = means(1);
        dyn_c_y = means(2);
        dyn_length = deltas(1);
        dyn_width = deltas(2);
        dyn_velocity = all_vels(dyn_obs_idx);
        dyn_heading_rad = 0;
        obj_mex(:,end+1) = [dyn_c_x; 
                            dyn_c_y; 
                            dyn_heading_rad;
                            dyn_velocity;
                            dyn_length;
                            dyn_width];
    end
    xlo = bounds(1) ; xhi = bounds(2) ;
    ylo = bounds(3) ; yhi = bounds(4) ;
    dx = xhi - xlo;
    dy = yhi - ylo;
    x_c = mean([xlo, xhi]);
    y_c = mean([ylo, yhi]);
    b_thick = 0.01;
    b_half_thick = b_thick / 2.0;

    % Top
    obj_mex(:,end+1) = [x_c; yhi+b_half_thick; 0; 0; dx; b_thick];
    % Bottom
    obj_mex(:,end+1) = [x_c; ylo-b_half_thick; 0; 0; dx; b_thick];
    % Right
    obj_mex(:,end+1) = [xhi+b_half_thick; y_c; 0; 0; b_thick; dy];
    % Left
    obj_mex(:,end+1) = [xlo-b_half_thick; y_c; 0; 0; b_thick; dy];
end
