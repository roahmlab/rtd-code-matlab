classdef simulator_armtd_ensemble < simulator
% Class: simulator
%
% S = simulator(agents, worlds, planners, varargin)
%
% Authors: Shreyas Kousik, Sean Vaskov, and Hannah Larson
% Created: a long, long time ago, on a laptop far, far away
% Updated: 8 Mar 2020
% Updated: 1 Dec 2021 by Patrick Holmes for ARMTD. Adding checks for input
% bounds, ultimate bound and joint limits.
%
% updated to run the planners as an ensemble and choose the results of the
% last planner just for analysis on the resulting optimizations.

%% properties
    properties (Access = public)
        stop_sim_when_input_exceeded = true;
        stop_sim_when_ultimate_bound_exceeded = true;
        stop_sim_when_joint_limit_exceeded = false;
    end

%% methods
    methods
    %% constructor
        function S = simulator_armtd_ensemble(varargin)
            % Constructor function: simulator
            %
            % Usage: S = simulator(agents, world, planners, varargin)
            %
            % This constructor takes in agents, which obey some physical
            % dynamics; world objects, which contains obstacles and goals;
            % and planners for the agent to perform receding-horizon traj.
            % planning in the worlds.
            
            S@simulator(varargin{:});
        end

    %% run simulation
        function summary = run(S,planner_indices)
            % Method: run
            %
            % This function simulates the agent in the provided world as it
            % attempts to reach the goal from its provided start position.
            
            S.vdisp('Running simulation')
            
            run_start_tic = tic ;

            % get simulation info
            t_max = S.max_sim_time ;
            iter_max = S.max_sim_iterations ;
            plot_in_loop_flag = S.plot_while_running ;

            % get world and planner indices
            world_indices = 1:length(S.worlds) ;
            
            if nargin < 2 || isempty(planner_indices)
                planner_indices = 1:length(S.planners) ;
            end

            % set up summaries
            LW = length(world_indices) ;
            summary = cell(1,LW) ;

        %% world loop
            for widx = world_indices
                S.vdisp('-------------------------------------------------',3,false)
                S.vdisp('-------------------------------------------------',4,false)
                S.vdisp(['Starting World ',num2str(widx)],1)
                W = S.get_world(widx) ;

                % set up summary objects
                L = length(planner_indices) ;
                agent_name_cell = cell(1,L) ;
                planner_name_cell = cell(1,L) ;
                planner_info_cell = cell(1,L) ;
                planner_new_info_cell = cell(1,L) ;
                agent_info_cell = cell(1,L) ;
                trajectory_cell = cell(1,L) ;
                total_real_time_cell = cell(1,L) ;
                total_iterations_cell = cell(1,L) ;
                planning_times_cell = cell(1,L) ;
                collision_check_cell = cell(1,L) ;
                input_check_cell = cell(1,L) ;
                ultimate_bound_check_cell = cell(1,L) ;
                joint_limit_check_cell = cell(1,L) ;
                goal_check_cell = cell(1,L) ;
                stop_check_cell = cell(1,L) ;
                total_simulated_time_cell = cell(1,L) ;
                control_input_cell = cell(1,L) ;
                control_input_time_cell = cell(1,L) ;
                t_plan_cell = cell(1,L) ;
                t_move_cell = cell(1,L) ;
                obstacles_cell = cell(1,L) ;
                
                world_start_tic = tic ;
                
            %% Ensemble the planners
                A = S.get_agent(1) ;

                % get agent and world ready
                W.reset() ;
                A.reset(W.start) ; 
                
                % preallocate for storing planning time spent
%                 planning_time_vec = cell(1,S.N_planners);
%                 for i=1:S.N_planners
%                     planning_time_vec{i} = nan(1,iter_max) ;
%                 end
                planning_time_vec = nan(1,iter_max) ;

                % check to make sure gif start is ready
                if S.save_gif
                    S.start_gif = true ;
                end

                % initialize plot
                if plot_in_loop_flag
                    S.plot(widx,1)
                end

                % reset the stop counter
                S.stop_count = 0 ;
                stop_check_vec = false(1,iter_max) ;
                
                % reset the crash and goal checks just in case
                collision_check = false ;
                goal_check = false ;
                input_check = false ;
                ultimate_bound_check = false;
                joint_limit_check = false;

                % start timing
                current_iteration = 1 ;
                planner_start_tic = tic ;
                t_cur = toc(planner_start_tic);
                
                % set up all planners
                for pidx = planner_indices
                    S.vdisp('---------------------------------------------',3,false)
                    S.vdisp(['Setting up Planner ',num2str(pidx)],1)

                    % get agent and planner
                    P = S.get_planner(pidx) ;

                    % get planner ready
                    agent_info = A.get_agent_info() ;
                    world_info = W.get_world_info(agent_info,P) ;
                    P.setup(agent_info,world_info) ;
                end

                % Set up the k_opt output
                k_opts_cell = {};
                k_opts_new_cell = {};
                k_opts_stats_cell = {};
                k_opts_new_stats_cell = {};

                %% simulation loop
                while current_iteration < (iter_max+1) && t_cur < t_max
                    S.vdisp('--------------------------------',3,false)
                    S.vdisp(['ITERATION ',num2str(current_iteration),' (t = ',num2str(A.time(end),'%0.2f'),')'],2,false)

                %% get agent info
                    agent_info = A.get_agent_info() ;

                %% get world info
                    % given the current state of the agent, query the world
                    % to get the surrounding obstacles
                    world_info = W.get_world_info(agent_info,P) ;

                %% replan
                    % given the current state and obstacles, query the
                    % current planner to get a control input
                    k_opts = zeros(7,L) * nan;
                    k_opts_new = zeros(7,L) * nan;

                    T_nom = cell(1,L);
                    U_nom = cell(1,L);
                    Z_nom = cell(1,L);
                    planner_info = cell(1,L);
                    t_plan_start_tic = tic ;
                    
                    parfor pidx = planner_indices
                        P = S.get_planner(pidx) ;
                        
                        if S.allow_replan_errors
                            [T_nom{pidx},U_nom{pidx},Z_nom{pidx},planner_info{pidx}] = P.replan(agent_info,world_info) ;
    %                             [planner_info] = P.replan(agent_info,world_info) ;
    %                             T_nom = planner_info.T{end};
    %                             U_nom = planner_info.U{end};
    %                             Z_nom = planner_info.Z{end};
                            k_opts(:,pidx) = P.info.k_opt{end};
                            k_opts_new(:,pidx) = P.new_info.k_opt{end};
                        else
                            try
                                [T_nom{pidx},U_nom{pidx},Z_nom{pidx},planner_info{pidx}] = P.replan(agent_info,world_info) ;
    %                                 [planner_info] = P.replan(agent_info,world_info) ;
    %                                 T_nom = planner_info.T{end};
    %                                 U_nom = planner_info.U{end};
    %                                 Z_nom = planner_info.Z{end};
                                k_opts(:,pidx) = P.info.k_opt{end};
                                k_opts_new(:,pidx) = P.new_info.k_opt{end};
                            catch
                                S.vdisp(['Planner ',num2str(pidx),' errored while ',...
                                         'replanning!'])
                                T_nom{pidx} = [] ; U_nom{pidx} = [] ; Z_nom{pidx} = [] ; planner_info{pidx} = [] ;
                            end
                        end
                    end
                    t_plan_spent = toc(t_plan_start_tic) ;
                    planning_time_vec(current_iteration) = t_plan_spent ;
                    S.vdisp(['Planning time all: ',num2str(t_plan_spent),' s'],4)
                    
                    k_opts_cell = [k_opts_cell, {k_opts}];
                    k_opts_stats_cell = [k_opts_stats_cell, ...
                        {[mean(k_opts,2),...
                          median(k_opts,2),...
                          std(k_opts,0,2),...
                          min(k_opts,[],2),...
                          max(k_opts,[],2),...
                          max(k_opts,[],2) - min(k_opts,[],2)]}];
                    k_opts_new_cell = [k_opts_new_cell, {k_opts_new}];
                    k_opts_new_stats_cell = [k_opts_new_stats_cell, ...
                        {[mean(k_opts_new,2),...
                          median(k_opts_new,2),...
                          std(k_opts_new,0,2),...
                          min(k_opts_new,[],2),...
                          max(k_opts_new,[],2),...
                          max(k_opts_new,[],2) - min(k_opts_new,[],2)]}];
                    % change references to simplify the following
                    pidx = 1;
                    T_nom = T_nom{pidx};
                    U_nom = U_nom{pidx};
                    Z_nom = Z_nom{pidx};
                    planner_info = planner_info{pidx};
                    P = S.get_planner(1) ;

                %% move agent
                    % update the agent using the current control input, so
                    % either stop if no control was returned, or move the
                    % agent if a valid input and time vector were returned
                    % if size(T_nom,2) < 2 || size(U_nom,2) < 2 || T_nom(end) == 0
                    if isnan(planner_info.k_opt{end})
                        S.vdisp('Stopping!',2)
                        % A.stop(P.t_move) ;
                        A.move(P.t_move,T_nom,U_nom,Z_nom,planner_info) ;

                        stop_check_vec(current_iteration) = true ;

                        % give planner a chance to recover from a stop
                        S.stop_count = S.stop_count + 1 ;
                        if S.stop_count > S.stop_threshold
                            break
                        end
                    else
                        S.stop_count = 0 ;

                        if ~isempty(P.t_move)
                            if P.t_move > T_nom(end)
                                S.vdisp(['The provided time vector for the ',...
                                    'agent input is shorter than the amount of ',...
                                    'time the agent must move at each ',...
                                    'planning iteration. The agent will only ',...
                                    'be moved for the duration of the ',...
                                    'provided time vector.'],3)

                                t_move = T_nom(end) ;
                            else
                                t_move = P.t_move ;
                            end
                        else
                            error(['Planner ',num2str(pidx),...
                                   '''s t_move property is empty!'])
                        end

                        A.move(t_move,T_nom,U_nom,Z_nom,planner_info) ;
                    end

                %% Note (22 July 2019)
                % Dynamic obstacles are treated as follows:
                %   1) W.get_world_info should return a prediction
                %   2) the agent is moved according to the prediction
                %   3) the world moves the obstacles (according to the
                %      agent's movement data if needed) and then checks
                %      for collisions in W.collision_check

                %% crash and goal check
                    % check if the agent is near the desired goal or if it
                    % crashed
                    S.vdisp('Checking if agent reached goal or crashed...',3)
                    agent_info = A.get_agent_info() ;
                    goal_check = W.goal_check(agent_info) ;
                    input_check = A.input_check(W.current_time) ; % must come before collision_check (which updates time)
                    if isprop(A.LLC, 'ultimate_bound')
                        ultimate_bound_check = A.LLC.ultimate_bound_check(A, W.current_time);  % must come before collision_check (which updates time)
                    else
                        ultimate_bound_check = false;
                    end
                    joint_limit_check = A.joint_limit_check(W.current_time); % must come before collision_check (which updates time)
                    collision_check = W.collision_check(agent_info,false) ;
                    
                    if isa(A,'multi_link_agent')
                        S.vdisp('Checking for self-intersection.',2)
                        collision_check = collision_check || A.self_intersection_flag ;
                    end
                                        
                    % update 20211201: add input bounds and ultimate_bound check
                    if input_check && S.stop_sim_when_input_exceeded
                        S.vdisp('Input exceeded!',2) ;
                        break
                    end
                    
                    if ultimate_bound_check && S.stop_sim_when_ultimate_bound_exceeded
                        S.vdisp('Ultimate bound exceeded!',2) ;
                        break
                    end
                    
                    if joint_limit_check && S.stop_sim_when_joint_limit_exceeded
                        S.vdisp('Joint limit exceeded!',2) ;
                        break
                    end

                    if collision_check && S.stop_sim_when_crashed
                        S.vdisp('Crashed!',2) ;
                        break
                    end

                    if goal_check
                        S.vdisp('Reached goal!',2) ;
                        break
                    end
                    

                    % plotting and animation
                    if plot_in_loop_flag
                        S.plot(widx,pidx)
                        if S.save_gif
                            error(['Saving a GIF is not yet supported for the simulator! ',...
                                'Use the agent''s animate method instead.'])
                        else
                            pause(S.plotting_pause_time) ;
                        end
                    end

                    % pause for user if needed
                    if S.manual_iteration
                        user_pause = tic ;
                        S.vdisp('Pausing for user. Press any key to continue.',2)
                        pause
                        user_pause = toc(user_pause) ;
                    else
                        user_pause = 0 ;
                    end

                    % iterate and increment time
                    S.vdisp(['END ITERATION ',num2str(current_iteration)],4,false)
                    current_iteration = current_iteration + 1 ;
                    t_cur = toc(planner_start_tic) - user_pause ;
                end
                sim_loop_time_spent = toc(planner_start_tic) ;
                S.vdisp(['Total time spent for all Planners ',...
                    num2str(pidx,'%0.2f'),': ',num2str(sim_loop_time_spent), 's'],5)

                % plot the last portion of the agent's trajectory after the
                % simulation ends
                if plot_in_loop_flag
                    S.plot(widx,pidx)
                end

                %% create summary (for the current planner)
                % get results at end of simulation
                S.vdisp('Compiling summary',3)
                Z = A.state ;
                T_nom = A.time ;
                U_nom = A.input ;
                TU = A.input_time ;
                agent_info = A.get_agent_info() ;
                
                if S.collision_check_full_traj_after_sim_flag
                    S.vdisp('Running final collision check.',4)
                    collision_check = W.collision_check(agent_info) ;
                end
                
                S.vdisp('Running final goal check',4)
                goal_check = W.goal_check(agent_info) ;
                agent_info_cell{pidx} = agent_info ;

                % Iterate for each planner
                for pidx = planner_indices
                    P = S.get_planner(pidx) ;

                    if S.save_planner_info
                        planner_info_cell{pidx} = S.planners{pidx}.info ;
                        planner_new_info_cell{pidx} = S.planners{pidx}.new_info ;
                    else
                        planner_info_cell{pidx} = 'no info saved' ;
                        planner_new_info_cell{pidx} = 'no info saved' ;
                    end
                end
                % results
                agent_name_cell = {A.name} ;
                planner_name_cell = {P.name} ;
                trajectory_cell = {Z} ;
                total_simulated_time_cell = {T_nom} ;
                control_input_cell = {U_nom} ;
                control_input_time_cell = {TU} ;
                total_real_time_cell = {sim_loop_time_spent} ;
                total_iterations_cell = {current_iteration} ;
                planning_times_cell = {planning_time_vec} ;
                collision_check_cell = {collision_check} ;
                input_check_cell = {input_check} ;
                ultimate_bound_check_cell = {ultimate_bound_check} ;
                joint_limit_check_cell = {joint_limit_check} ;
                goal_check_cell = {goal_check} ;
                stop_check_cell = {stop_check_vec} ;
                t_plan_cell = {P.t_plan} ;
                t_move_cell = {P.t_move} ;
                obstacles_cell = {W.obstacles};
                
                if goal_check
                    S.vdisp('In final check, agent reached goal!')
                end
                
                if input_check
                    S.vdisp('In final check, agent exceeded inputs!')
                end
                
                if ultimate_bound_check
                    S.vdisp('In final check, agent exceeded ultimate bound!')
                end
                
                if joint_limit_check
                    S.vdisp('In final check, agent exceeded joint limits!')
                end
                
                if collision_check
                    S.vdisp('In final check, agent crashed!')
                end
                
                %S.vdisp(['Finished Planner ',num2str(pidx)],1)
                %S.vdisp('---------------------------------------------',3,false)

                S.vdisp(['World ',num2str(widx),' complete! Generating summary.'])
                summary{widx} = struct('agent_name',agent_name_cell,...
                                 'planner_name',planner_name_cell,...
                                 'trajectory',trajectory_cell,...
                                 'total_real_time',total_real_time_cell,...
                                 'total_iterations',total_iterations_cell,...
                                 'planning_time',planning_times_cell,...
                                 'collision_check',collision_check_cell,...
                                 'input_check',input_check_cell,...
                                 'ultimate_bound_check', ultimate_bound_check_cell,...
                                 'joint_limit_check', joint_limit_check_cell,...
                                 'goal_check',goal_check_cell,...
                                 'stop_check',stop_check_cell,...
                                 'total_simulated_time',total_simulated_time_cell,...
                                 'control_input',control_input_cell,...
                                 'control_input_time',control_input_time_cell,...
                                 'agent_info',{agent_info_cell{1}},...    
                                 'planner_info',{planner_info_cell{1}},...
                                 'obstacles',obstacles_cell,...
                                 'planner_indices',planner_indices,...
                                 'N_obstacles',W.N_obstacles,...
                                 't_plan',t_plan_cell,...
                                 't_move',t_move_cell,...
                                 't_max',t_max,...
                                 'iter_max',iter_max,...
                                 'start',W.start,...
                                 'goal',W.goal,...
                                 'bounds',W.bounds,...
                                 'notes','',...
                                 'k_opts', {k_opts_cell}, ...
                                 'k_opts_new', {k_opts_new_cell},...
                                 'k_opts_stats', {k_opts_stats_cell}, ...
                                 'k_opts_new_stats', {k_opts_new_stats_cell}) ;
                             
                 S.vdisp(['Finished with World ',num2str(widx)],3)
                 S.vdisp(['Total time spent for World ',num2str(widx),': ',...
                     num2str(toc(world_start_tic),'%0.2f'),' s'],3) ;
                 S.vdisp('------------------------------------------------',3,false)
                 S.vdisp('------------------------------------------------',4,false)
            end
            
            S.vdisp(['Total time spent running simulator: ', num2str(toc(run_start_tic),'%0.2f'),' s'],3) ;

            % clean up summary if only one world was run
            if LW == 1
                summary = summary{1} ;
            end
            
            S.simulation_summary = summary ;

            if nargout < 1
                clear summary ;
                S.vdisp('Simulation summary stored in simulator_obj.simulation_summary.',1)
            end
            
            S.vdisp('Simulation complete!')
        end
    end
end
