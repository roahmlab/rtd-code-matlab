classdef uarmtd_planner_wrapped_comparison < robot_arm_generic_planner
    % UARMTD_PLANNER implements new polynomial zonotope collision-avoidance
    % and input constraints
    
    properties
        % processed
        agent
        planner
        latest_trajectory
        new_info
        wait_on_first_run = true
        random_init = false
        comparison_delta = 1e-8

        % housekeeping
        time_discretization = 0.01;        
        trajopt_start_tic;
        iter = 0;
        first_iter_pause_flag = true ;
        
        % jrs and agent info
        jrs_info;
        agent_info;

        % cost type
        use_q_plan_for_cost = false; % otherwise use q_stop (q at final time)

        % constraints:
        constraints = {}; % cell will contain functions of $k$ for evaluating constraints
        grad_constraints = {}; % cell will contain functions of $k$ for evaluating gradients
        obs_constraints = {};
        grad_obs_constraints = {};

        % smooth obstacle constraints:
        smooth_obs_constraints;
        smooth_obs_constraints_A;
        smooth_obs_lambda_index;
        
        % for turning on/off constraint types
        save_FO_zono_flag = true;
        input_constraints_flag = true;
        grasp_constraints_flag = false; % must have input constraints turned on!
        smooth_obstacle_constraints_flag = false;
        use_robust_input = true; % turn this off to only consider nominal passivity-based controller

        % for obstacle avoidance:
        combs;

        % for JRSs and trajectories:
        taylor_degree = 1;
        traj_type = 'orig'; % choose 'orig' for original ARMTD, or 'bernstein'
    end
    
    methods
        function P = uarmtd_planner_wrapped_comparison(varargin)
            t_move = 0.5;
            lookahead_distance = 0.4;
            HLP = robot_arm_straight_line_HLP( );
            P@robot_arm_generic_planner('lookahead_distance', lookahead_distance, 't_move', t_move, 'HLP', HLP, ...
                varargin{:}) ;
            
            % hard code planning time...
            P.t_plan = 0.5;

            % init info object
            P.init_info()
            
            % initialize combinations (for obstacle avoidance constraints)
            P.combs.maxcombs = 200;
            P.combs.combs = generate_combinations_upto(200);
            
            % Here's where the wrapper really starts
            trajOptProps = TrajOptProps;
            trajOptProps.planTime = P.t_move;
            trajOptProps.horizonTime = P.t_stop;
            trajOptProps.doTimeout = false;
            trajOptProps.timeoutTime = P.t_move;
            trajOptProps.randomInit = P.random_init;
            if P.use_q_plan_for_cost
                trajOptProps.timeForCost = P.t_move;
            else
                trajOptProps.timeForCost = P.t_stop;
            end
            
            robotInfo = ArmRobotInfo;
            robotInfo.params = P.agent.params;
            robotInfo.link_poly_zonotopes = P.agent.link_poly_zonotopes;
            robotInfo.LLC_info = {};
            robotInfo.LLC_info.ultimate_bound = P.agent.LLC.ultimate_bound;
            robotInfo.LLC_info.ultimate_bound_position = P.agent.LLC.ultimate_bound_position;
            robotInfo.LLC_info.ultimate_bound_velocity = P.agent.LLC.ultimate_bound_velocity;
            robotInfo.LLC_info.alpha_constant = P.agent.LLC.alpha_constant;
            robotInfo.joint_input_limits = P.agent.joint_input_limits;
            
            worldInfo = WorldInfo;
            
            input_constraints_flag = P.input_constraints_flag;
            use_robust_input = P.use_robust_input;
            smooth_obs = P.smooth_obstacle_constraints_flag;
            
            P.planner = ArmourPlanner( ...
                    trajOptProps, robotInfo, worldInfo, ...
                    input_constraints_flag, use_robust_input, smooth_obs, P.traj_type ...
                );
        end
        
        function init_info(P)
            P.info = struct('T',[],'U',[],'Z',[],'waypoint',[],...
                'obstacles',[],'q_0',[],'q_dot_0',[],'k_opt',[],...
                'desired_trajectory', [], 't_move', [], ...
                'FO_zono', [], 'sliced_FO_zono', [], 'planning_time', [], 'error_count', []) ;
            P.new_info = struct('T',[],'U',[],'Z',[],'waypoint',[],...
                'obstacles',[],'q_0',[],'q_dot_0',[],'k_opt',[],...
                'desired_trajectory', [], 't_move', [], ...
                'FO_zono', [], 'sliced_FO_zono', [], 'planning_time', []) ;
        end
        
        function [T, U, Z, info] = replan(P,agent_info,world_info)
            P.vdisp('Replanning!',5)
            
            % get current state of robot
            P.agent_info = agent_info;

            q_0 = agent_info.reference_state(P.arm_joint_state_indices, end) ;
            q_dot_0 = agent_info.reference_state(P.arm_joint_speed_indices, end) ;
            q_ddot_0 = agent_info.reference_acceleration(:, end);
            % q_ddot_0 = zeros(size(q_0)); % need to pass this in for bernstein!!!

            % if bounds are +- inf, set to small/large number
            joint_limit_infs = isinf(agent_info.joint_state_limits) ;
            speed_limit_infs = isinf(agent_info.joint_speed_limits) ;
            input_limit_infs = isinf(agent_info.joint_input_limits) ;

            agent_info.joint_state_limits(1,joint_limit_infs(1,:)) = -200*pi ;
            agent_info.joint_state_limits(2,joint_limit_infs(2,:)) = +200*pi ;            
            agent_info.joint_speed_limits(1,speed_limit_infs(1,:)) = -200*pi ;
            agent_info.joint_speed_limits(2,speed_limit_infs(2,:)) = +200*pi ;
            agent_info.joint_input_limits(1,input_limit_infs(1,:)) = -200*pi ;
            agent_info.joint_input_limits(2,input_limit_infs(2,:)) = +200*pi ;

            % generate a waypoint in configuration space
            P.vdisp('Generating cost function',6)
            if P.first_iter_pause_flag && P.iter == 0
                % Create initial trajectory
                P.vdisp('Generating initial trajectory for new planner', 6)
                robotState = ArmRobotState;
                robotState.time = 0.0;
                robotState.q = q_0;
                robotState.q_dot = q_dot_0;
                robotState.q_ddot = q_ddot_0;
                rs = P.planner.jrsHandle.getReachableSet(robotState, false);
                traj = P.planner.trajectoryFactory(robotState,{rs});
                traj.setTrajectory(zeros(traj.param_shape,1));
                P.new_info.desired_trajectory = [P.new_info.desired_trajectory, {@(t) unwrap_traj(traj.getCommand(t))}];
                P.latest_trajectory = traj;

                if P.wait_on_first_run
                    P.vdisp('Press Enter to Continue:')
                    pause;
                end
            end
            P.iter = P.iter + 1;
            waypoint_time = tic;
            q_des = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
            if isempty(q_des)
                P.vdisp('Waypoint creation failed! Using global goal instead.', 3)
                q_des = P.HLP.goal ;
            end
                        
            waypoint_time = toc(waypoint_time);
            P.vdisp([num2str(waypoint_time), 's for waypoint generation']);
            
            
            %% New planner
            new_planner_time = tic;
            
            P.vdisp('Running Wrapped Planner!',6)
            worldState = WorldState;
            worldState.obstacles = world_info.obstacles;
            
            robotState = ArmRobotState;
            robotState.time = agent_info.time(end);
            robotState.q = q_0;
            robotState.q_dot = q_dot_0;
            robotState.q_ddot = q_ddot_0;
            
            waypoint = q_des;
            
            [trajectory, plan_info] = P.planner.planTrajectory(robotState, worldState, waypoint);
            FO = plan_info.rsInstances{2}.FO;
            jrsinfo = plan_info.rsInstances{1}.jrs_info;
            
            
            try
                % Update k_opt for info struct
                k_opt = trajectory.getTrajParams();
                P.vdisp('New trajectory found!',3);
                P.latest_trajectory = trajectory;
                trajopt_failed = false;
            catch
                % If invalid trajectory, just make k_opt nan, and don't
                % update the latest trajectory.
                P.vdisp('Unable to find new trajectory!',3)
                k_opt = nan;
                trajopt_failed = true;
            end
            
            new_planner_time = toc(new_planner_time);
            
            P.new_info.desired_trajectory = [P.new_info.desired_trajectory, {@(t) unwrap_traj(P.latest_trajectory.getCommand(robotState.time + t))}];
            P.new_info.t_move = [P.new_info.t_move, {P.t_move}];
            P.new_info.waypoint = [P.new_info.waypoint, {q_des}] ;
            P.new_info.obstacles = [P.new_info.obstacles, {world_info.obstacles}] ;
            P.new_info.q_0 = [P.new_info.q_0, {q_0}] ;
            P.new_info.q_dot_0 = [P.new_info.q_dot_0, {q_dot_0}] ;
            P.new_info.k_opt = [P.new_info.k_opt, {k_opt}] ;
            if P.save_FO_zono_flag
                for i = 1:jrsinfo.n_t
                    for j = 1:P.agent_info.params.pz_nominal.num_bodies
                        FO_zono{i}{j} = zonotope(FO{i}{j});
                        if trajopt_failed
                            % no safe slice
                            sliced_FO_zono{i}{j} = [];
                        else
                            % slice and save
                            fully_sliceable_tmp = polyZonotope_ROAHM(FO{i}{j}.c, FO{i}{j}.G, [], FO{i}{j}.expMat, FO{i}{j}.id);
                            sliced_FO_zono{i}{j} = zonotope([slice(fully_sliceable_tmp, k_opt), FO{i}{j}.Grest]);
                        end
                    end
                end
                P.new_info.FO_zono = [P.new_info.FO_zono, {FO_zono}];
                P.new_info.sliced_FO_zono = [P.new_info.sliced_FO_zono, {sliced_FO_zono}];
            end
            P.new_info.planning_time = [P.new_info.planning_time, new_planner_time];
            P.vdisp([num2str(new_planner_time), 's for new planner']);
            
            %% Old planner
            old_planner_time = tic;
            % get current obstacles and create constraints
            P.vdisp('Generating constraints',6)
            [P, FO] = P.generate_constraints(q_0, q_dot_0, q_ddot_0, world_info.obstacles);
            
            % optimize
            P.vdisp('Replan is calling trajopt!',8)
            [k_opt, trajopt_failed] = P.trajopt(q_0, q_dot_0, q_ddot_0, q_des);
            if P.smooth_obstacle_constraints_flag
                k_opt = k_opt(1:P.agent_info.params.pz_nominal.num_q);
            end

            % process result            
            if ~trajopt_failed
                P.vdisp('New trajectory found!',3);
            else % no safe trajectory parameter found:
                P.vdisp('Unable to find new trajectory!',3)
                k_opt = nan;
            end
            old_planner_time = toc(old_planner_time);
            P.vdisp([num2str(old_planner_time), 's for old planner']);
            
            % save info
            P.info.desired_trajectory = [P.info.desired_trajectory, {@(t) P.desired_trajectory(q_0, q_dot_0, q_ddot_0, t, k_opt)}];
            P.info.t_move = [P.info.t_move, {P.t_move}];
            P.info.waypoint = [P.info.waypoint, {q_des}] ;
            P.info.obstacles = [P.info.obstacles, {world_info.obstacles}] ;
            P.info.q_0 = [P.info.q_0, {q_0}] ;
            P.info.q_dot_0 = [P.info.q_dot_0, {q_dot_0}] ;
            P.info.k_opt = [P.info.k_opt, {k_opt}] ;
            if P.save_FO_zono_flag
                for i = 1:P.jrs_info.n_t
                    for j = 1:P.agent_info.params.pz_nominal.num_bodies
                        FO_zono{i}{j} = zonotope(FO{i}{j});
                        if trajopt_failed
                            % no safe slice
                            sliced_FO_zono{i}{j} = [];
                        else
                            % slice and save
                            fully_sliceable_tmp = polyZonotope_ROAHM(FO{i}{j}.c, FO{i}{j}.G, [], FO{i}{j}.expMat, FO{i}{j}.id);
                            sliced_FO_zono{i}{j} = zonotope([slice(fully_sliceable_tmp, k_opt), FO{i}{j}.Grest]);
                        end
                    end
                end
                P.info.FO_zono = [P.info.FO_zono, {FO_zono}];
                P.info.sliced_FO_zono = [P.info.sliced_FO_zono, {sliced_FO_zono}];
            end
            P.info.planning_time = [P.info.planning_time, old_planner_time];

            %% create outputs:
            T = 0:P.time_discretization:P.t_stop ;
            U = zeros(agent_info.n_inputs, length(T));
            Z = zeros(agent_info.n_states, length(T));
            error_count = 0;
            for i = 1:length(T)
                [q_tmp, qd_tmp, ~] = P.info.desired_trajectory{end}(T(i));
                % COMPARE
                [q_tmp_comp, qd_tmp_comp, ~] = P.new_info.desired_trajectory{end}(T(i));
                if i < length(T)/2 && ~(norm(q_tmp-q_tmp_comp) < P.comparison_delta && norm(qd_tmp-qd_tmp_comp) < P.comparison_delta)
                    % Put breakpoint here for comparison!
                    P.vdisp('Disparity between old and new planner detected!');
                    P.vdisp('Using old planner values!');
                    error_count = error_count + 1;
                end
                Z(agent_info.joint_state_indices, i) = q_tmp;
                Z(agent_info.joint_speed_indices, i) = qd_tmp;
            end
            P.info.error_count = [P.info.error_count, error_count];
            info = P.info;
            info.new_info = P.new_info;

        end
        
        function [P, FO] = generate_constraints(P, q_0, q_dot_0, q_ddot_0, O)
            %% get JRSs:
            [q_des, dq_des, ddq_des, q, dq, dq_a, ddq_a, R_des, R_t_des, R, R_t, jrs_info] = create_jrs_online(q_0, q_dot_0, q_ddot_0,...
                P.agent_info.params.pz_nominal.joint_axes, P.taylor_degree, P.traj_type, P.use_robust_input);
            P.jrs_info = jrs_info;

            %% create FO and input poly zonotopes:
            % set up zeros and overapproximation of r
            for j = 1:jrs_info.n_q
                zero_cell{j, 1} = polyZonotope_ROAHM(0); 
                r{j, 1} = polyZonotope_ROAHM(0, [], P.agent_info.LLC_info.ultimate_bound);
            end
            
            % get forward kinematics and forward occupancy
            for i = 1:jrs_info.n_t
               [R_w{i, 1}, p_w{i, 1}] = pzfk(R{i, 1}, P.agent_info.params.pz_nominal); 
               for j = 1:P.agent_info.params.pz_nominal.num_bodies
                  FO{i, 1}{j, 1} = R_w{i, 1}{j, 1}*P.agent_info.link_poly_zonotopes{j, 1} + p_w{i, 1}{j, 1}; 
                  FO{i, 1}{j, 1} = reduce(FO{i, 1}{j, 1}, 'girard', P.agent_info.params.pz_interval.zono_order);
                  FO{i, 1}{j, 1} = remove_dependence(FO{i, 1}{j, 1}, jrs_info.k_id(end));
               end
            end

            % get nominal inputs, disturbances, possible lyapunov functions:
            % can possibly speed up by using tau_int - tau_int to get disturbance.
            if P.input_constraints_flag
                for i = 1:jrs_info.n_t
                    tau_nom{i, 1} = poly_zonotope_rnea(R{i}, R_t{i}, dq{i}, dq_a{i}, ddq_a{i}, true, P.agent_info.params.pz_nominal);
                    if P.use_robust_input
                        [tau_int{i, 1}, f_int{i, 1}, n_int{i, 1}] = poly_zonotope_rnea(R{i}, R_t{i}, dq{i}, dq_a{i}, ddq_a{i}, true, P.agent_info.params.pz_interval);
                        for j = 1:jrs_info.n_q
                            w{i, 1}{j, 1} = tau_int{i, 1}{j, 1} - tau_nom{i, 1}{j, 1};
                            w{i, 1}{j, 1} = reduce(w{i, 1}{j, 1}, 'girard', P.agent_info.params.pz_interval.zono_order);
                        end
                        V_cell = poly_zonotope_rnea(R{i}, R_t{i}, zero_cell, zero_cell, r, false, P.agent_info.params.pz_interval);
                        V{i, 1} = 0;
                        for j = 1:jrs_info.n_q
                            V{i, 1} = V{i, 1} + 0.5.*r{j, 1}.*V_cell{j, 1};
                            V{i, 1} = reduce(V{i, 1}, 'girard', P.agent_info.params.pz_interval.zono_order);
                        end
                        V_diff{i, 1} = V{i, 1} - V{i, 1};
                        V_diff{i, 1} = reduce(V_diff{i, 1}, 'girard', P.agent_info.params.pz_interval.zono_order);
                        V_diff_int{i, 1} = interval(V_diff{i, 1});
                    end
                end

                % % get max effect of disturbance r'*w... couple of ways to do this
                % % can overapproximate r and compute r'*w as a PZ
                % % can slice w first, then get the norm?
                % for i = 1:jrs_info.n_t
                %     r_dot_w{i, 1} = 0;
                %     for j = 1:jrs_info.n_q
                %         r_dot_w{i, 1} = r_dot_w{i, 1} + r{j, 1}.*w{i, 1}{j, 1};
                %         r_dot_w{i, 1} = reduce(r_dot_w{i, 1}, 'girard', P.params.pz_interval.zono_order);
                %     end
                % end
                % can get ||w|| <= ||\rho(\Phi)||, and compute the norm using interval arithmetic
                if P.use_robust_input
                    for i = 1:jrs_info.n_t
                        for j = 1:jrs_info.n_q
                            w_int{i, 1}(j, 1) = interval(w{i, 1}{j, 1});
                        end
                        rho_max{i, 1} = norm(max(abs(w_int{i, 1}.inf), abs(w_int{i, 1}.sup)));
                    end

                    % compute robust input bound tortatotope:
                    for i = 1:jrs_info.n_t
                        v_norm{i, 1} = (P.agent_info.LLC_info.alpha_constant*V_diff_int{i, 1}.sup).*(1/P.agent_info.LLC_info.ultimate_bound) + rho_max{i, 1};
    %                     v_norm{i, 1} = reduce(v_norm{i, 1}, 'girard', P.agent_info.params.pz_interval.zono_order);
                    end
                else
                    for i = 1:jrs_info.n_t
                        v_norm{i, 1} = 0;
                    end
                end

                % compute total input tortatotope
                for i = 1:jrs_info.n_t
                    for j = 1:jrs_info.n_q
                        u_ub_tmp = tau_nom{i, 1}{j, 1} + v_norm{i, 1};
                        u_lb_tmp = tau_nom{i, 1}{j, 1} - v_norm{i, 1};
                        u_ub_tmp = remove_dependence(u_ub_tmp, jrs_info.k_id(end));
                        u_lb_tmp = remove_dependence(u_lb_tmp, jrs_info.k_id(end));
                        u_ub_buff = sum(abs(u_ub_tmp.Grest));
                        u_lb_buff = -sum(abs(u_lb_tmp.Grest));
                        u_ub{i, 1}{j, 1} = polyZonotope_ROAHM(u_ub_tmp.c + u_ub_buff, u_ub_tmp.G, [], u_ub_tmp.expMat, u_ub_tmp.id) - P.agent_info.joint_input_limits(2, j);
                        u_lb{i, 1}{j, 1} = -1*polyZonotope_ROAHM(u_lb_tmp.c + u_lb_buff, u_lb_tmp.G, [], u_lb_tmp.expMat, u_lb_tmp.id) + P.agent_info.joint_input_limits(1, j);
                    end
                end
                
                % grasp constraints:
                if P.grasp_constraints_flag
                    % ASSUMING SURFACE NORMAL IS POSITIVE Z-DIRECTION
                    % fill this out still
                end
            end

            %% start making constraints
            P.constraints = {};
            P.grad_constraints = {};
            if ~P.smooth_obstacle_constraints_flag
                P.obs_constraints = {};
                P.grad_obs_constraints = {};
            else
                P.smooth_obs_constraints = {};
                P.smooth_obs_constraints_A = {};
                P.smooth_obs_lambda_index = {};
            end

            % obstacle avoidance constraints
            for i = 1:jrs_info.n_t
                for j = 1:P.agent_info.params.pz_nominal.num_bodies
                    for o = 1:length(O) % for each obstacle
                        
                        % first, check if constraint is necessary
                        O_buf = [O{o}.Z, FO{i, 1}{j, 1}.G, FO{i, 1}{j, 1}.Grest];
                        [A_obs, b_obs] =  polytope_PH(O_buf, P.combs); % get polytope form
                        if ~(all(A_obs*FO{i, 1}{j, 1}.c - b_obs <= 0, 1))
                            continue;
                        end
                        
                        % reduce FO so that polytope_PH has fewer
                        % directions to consider
                        FO{i, 1}{j, 1} = reduce(FO{i, 1}{j, 1}, 'girard', 3);
                        
                        % now create constraint
                        FO_buf = FO{i, 1}{j, 1}.Grest; % will buffer by non-sliceable gens
                        O_buf = [O{o}.Z, FO_buf]; % describes buffered obstacle zonotope
                        [A_obs, b_obs] = polytope_PH(O_buf, P.combs); % get polytope form

                        % constraint PZ:
                        FO_tmp = polyZonotope_ROAHM(FO{i, 1}{j, 1}.c, FO{i, 1}{j, 1}.G, [], FO{i, 1}{j, 1}.expMat, FO{i, 1}{j, 1}.id);
                        obs_constraint_pz = A_obs*FO_tmp - b_obs;

                        % turn into function
                        obs_constraint_pz_slice = @(k) slice(obs_constraint_pz, k);

                        % add gradients
                        grad_obs_constraint_pz = grad(obs_constraint_pz, P.jrs_info.n_q);
                        grad_obs_constraint_pz_slice = @(k) cellfun(@(C) slice(C, k), grad_obs_constraint_pz, 'UniformOutput', false);
                        
                        % save
                        if ~P.smooth_obstacle_constraints_flag
                            P.obs_constraints{end+1, 1} = @(k) P.evaluate_obs_constraint(obs_constraint_pz_slice, grad_obs_constraint_pz_slice, k);
                        else
                            P.smooth_obs_constraints_A{end+1, 1} = A_obs;
                            P.smooth_obs_constraints{end+1, 1} = @(k, lambda) P.evaluate_smooth_obs_constraint(obs_constraint_pz_slice, grad_obs_constraint_pz_slice, k, lambda);
                            if isempty(P.smooth_obs_lambda_index)
                                P.smooth_obs_lambda_index{end+1, 1} = (1:size(obs_constraint_pz.c, 1))';
                            else
                                P.smooth_obs_lambda_index{end+1, 1} = P.smooth_obs_lambda_index{end}(end, 1) + (1:size(obs_constraint_pz.c, 1))';
                            end
                        end
                    end
                end
            end 

            % input constraints
            if P.input_constraints_flag
                for i = 1:jrs_info.n_t
                    for j = 1:jrs_info.n_q

                        % first check if constraints are necessary, then add
                        u_ub_int = interval(u_ub{i, 1}{j, 1});
                        if ~(u_ub_int.sup < 0)
                            % no constraint necessary
%                             continue;
%                         else
                            % add constraint and gradient
                            fprintf('ADDED UPPER BOUND INPUT CONSTRAINT ON JOINT %d \n', j);
                            P.constraints{end+1, 1} = @(k) slice(u_ub{i, 1}{j, 1}, k);
                            grad_u_ub = grad(u_ub{i, 1}{j, 1}, P.jrs_info.n_q);
                            P.grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_u_ub);
                        end

                        u_lb_int = interval(u_lb{i, 1}{j, 1});
                        if ~(u_lb_int.sup < 0)
                            % no constraint necessary
%                             continue;
%                         else
                            % add constraint and gradient
                            fprintf('ADDED LOWER BOUND INPUT CONSTRAINT ON JOINT %d \n', j);
                            P.constraints{end+1, 1} = @(k) slice(u_lb{i, 1}{j, 1}, k);
                            grad_u_lb = grad(u_lb{i, 1}{j, 1}, P.jrs_info.n_q);
                            P.grad_constraints{end+1, 1} = @(k) cellfun(@(C) slice(C, k), grad_u_lb);
                        end
                    end
                end
            end

            % more constraints to add:
            % 1) joint limit constraints
            % 2) joint velocity constraints
            % 3) orientation constraints
            % 4) self intersection constraints
            % 5) joint position (in workspace) constraints?
        end

        function [h_obs, grad_h_obs] = evaluate_obs_constraint(P, c, grad_c, k) 
            % made a separate function to handle the obstacle constraints,
            % because the gradient requires knowing the index causing
            % the max of the constraints
            [h_obs, max_idx] = max(c(k));
            h_obs = -h_obs;
            grad_eval = grad_c(k);
            grad_h_obs = zeros(length(k), 1);
            for i = 1:length(k)
                grad_h_obs(i, 1) = -grad_eval{i}(max_idx, :);
            end
        end

        function [h_obs, grad_h_obs] = evaluate_smooth_obs_constraint(P, c, grad_c, k, lambda) 
            % for evaluating smooth obs constraints, where lambda is introduced
            % as extra decision variables to avoid taking a max.

            % evaluate constraint: (A*FO - b)'*lambda
            h_obs = -c(k)'*lambda;

            % evaluate gradient w.r.t. k... 
            % grad_c(k) gives n_k x 1 cell, each containing
            % an N x 1 vector, where N is the number of rows of A.
            % take the dot product of each cell with lambda
            grad_eval = grad_c(k);
            grad_h_obs = zeros(length(k), 1);
            for i = 1:length(k)
                grad_h_obs(i, 1) = -grad_eval{i}'*lambda;
            end

            % evaluate gradient w.r.t. lambda...
            % this is just (A*FO-b)
            grad_h_obs = [grad_h_obs; -c(k)];
        end

        
        function [k_opt, trajopt_failed] = trajopt(P, q_0, q_dot_0, q_ddot_0, q_des)
            % use fmincon to optimize the cost subject to constraints
            P.vdisp('Running trajopt', 3)
            
            P.trajopt_start_tic = tic ;
            n_k = P.jrs_info.n_k;
            if P.smooth_obstacle_constraints_flag && ~isempty(P.smooth_obs_lambda_index)
                cost_func = @(x) P.eval_cost(x(1:n_k), q_0, q_dot_0, q_ddot_0, q_des);
                constraint_func = @(x) P.eval_smooth_constraint(x(1:n_k), x(n_k+1:end));
                lb_k = -ones(n_k, 1);
                ub_k = ones(n_k, 1);
                lb_lambda = zeros(P.smooth_obs_lambda_index{end}(end), 1);
                ub_lambda = ones(P.smooth_obs_lambda_index{end}(end), 1);
                lb = [lb_k; lb_lambda];
                ub = [ub_k; ub_lambda];
            else
                cost_func = @(k) P.eval_cost(k, q_0, q_dot_0, q_ddot_0, q_des);
                constraint_func = @(k) P.eval_constraint(k);
                lb = -ones(n_k, 1);
                ub = ones(n_k, 1);
            end

            if P.random_init
                initial_guess = rand_range(lb, ub);
            else
                initial_guess = lb*0.0;
            end
           
            options = optimoptions('fmincon','SpecifyConstraintGradient',true);
%             options = optimoptions('fmincon','SpecifyConstraintGradient',true, 'CheckGradients', true);
            [k_opt, ~, exitflag, ~] = fmincon(cost_func, initial_guess, [], [], [], [], lb, ub, constraint_func, options) ;
            
            trajopt_failed = exitflag <= 0 ;
        end
        
        function [cost] = eval_cost(P, k, q_0, q_dot_0, q_ddot_0, q_des)
            if P.use_q_plan_for_cost
                q_plan = P.desired_trajectory(q_0, q_dot_0, q_ddot_0, P.t_plan, k);
                cost = sum((q_plan - q_des).^2);
            else
                % use final position as cost
                q_stop = P.desired_trajectory(q_0, q_dot_0, q_ddot_0, P.t_stop, k);
                cost = sum((q_stop - q_des).^2);
            end
        end

        function [h, heq, grad_h, grad_heq] = eval_constraint(P, k)
            n_obs_c = length(P.obs_constraints);
            n_c = length(P.constraints);

            h = zeros(n_c + n_obs_c, 1);
            grad_h = zeros(length(k), n_c + n_obs_c);
            
            for i = 1:n_obs_c
                [h_i, grad_h_i] = P.obs_constraints{i}(k);
                h(i) = h_i;
                grad_h(:, i) = grad_h_i;
            end

            for i = 1:n_c
                h(i + n_obs_c) = P.constraints{i}(k);
                grad_h(:, i + n_obs_c) = P.grad_constraints{i}(k);
            end

            grad_heq = [];
            heq = [];
        end

        function [h, heq, grad_h, grad_heq] = eval_smooth_constraint(P, k, lambda)

            n_obs_c = length(P.smooth_obs_constraints);
            n_k = length(k);
            n_lambda = length(lambda);

            % max_lambda_index = P.smooth_obs_lambda_index{end}(end);

            % number of constraints: 
            % - 1 obstacle avoidance per obstacle
            % - 1 sum lambdas for obstacle = 1
            % - n_lambda lambda \in {0, 1}

            h = zeros(2*n_obs_c, 1);
            grad_h = zeros(n_k + n_lambda, 2*n_obs_c);

            for i = 1:n_obs_c
                lambda_idx = P.smooth_obs_lambda_index{i};
                lambda_i = lambda(lambda_idx, 1);% pull out correct lambdas!!
                [h_i, grad_h_i] = P.smooth_obs_constraints{i}(k, lambda_i);

                % obs avoidance constraints:
                h(i, 1) = h_i;
                grad_h(1:n_k, i) = grad_h_i(1:n_k, 1);
                grad_h(n_k + lambda_idx, i) = grad_h_i(n_k+1:end, 1);

                % sum lambdas for this obstacle constraint >= 1
                % sum lambdas for this obstacle constraint <= 1?
%                 h(n_obs_c + i, 1) = 1 - sum(lambda_i, 1); 
%                 grad_h(n_k + lambda_idx, n_obs_c + i) = -ones(length(lambda_i), 1);

                % from Borrelli paper
                h(n_obs_c + i, 1) = norm(P.smooth_obs_constraints_A{i}'*lambda_i, 2) - 1;
                % implement gradient here!!!
                A_bar = P.smooth_obs_constraints_A{i}*P.smooth_obs_constraints_A{i}';
                grad_h(n_k + lambda_idx, n_obs_c + i) = 0.5*(lambda_i'*A_bar*lambda_i)^(-0.5)*2*A_bar*lambda_i;
                
            end

            % lambda \in {0, 1} for each lambda
%             heq = lambda.*(lambda - 1);
%             grad_heq = zeros(n_k + n_lambda, n_lambda);
%             grad_heq(n_k+1:end, :) = diag(2*lambda - 1);

             heq = [];
             grad_heq = [];
        end

        function [q_des, qd_des, qdd_des] = desired_trajectory(P, q_0, q_dot_0, q_ddot_0, t, k)
            % at a given time t and traj. param k value, return
            % the desired position, velocity, and acceleration.
            switch P.traj_type
            case 'orig'
                t_plan = P.t_plan;
                t_stop = P.t_stop;
                k_scaled = P.jrs_info.c_k + P.jrs_info.g_k.*k;
                
                if ~isnan(k)
                    if t <= t_plan
                        % compute first half of trajectory
                        q_des = q_0 + q_dot_0*t + (1/2)*k_scaled*t^2;
                        qd_des = q_dot_0 + k_scaled*t;
                        qdd_des = k_scaled;
                    else
                        % compute trajectory at peak
                        q_peak = q_0 + q_dot_0*t_plan + (1/2)*k_scaled*t_plan^2;
                        qd_peak = q_dot_0 + k_scaled*t_plan;

                        % compute second half of trajectory
                        q_des = q_peak + qd_peak*(t-t_plan) + (1/2)*((0 - qd_peak)/(t_stop - t_plan))*(t-t_plan)^2;
                        qd_des = qd_peak + ((0 - qd_peak)/(t_stop - t_plan))*(t-t_plan);
                        qdd_des = (0 - qd_peak)/(t_stop - t_plan);
                    end
                else
                    % bring the trajectory to a stop in t_plan seconds
                    % trajectory peaks at q_0
                    q_peak = q_0;
                    qd_peak = q_dot_0;
                    
                    if t <= t_plan && ~all(q_dot_0 == 0) % we're braking!
                        q_des = q_peak + qd_peak*t + (1/2)*((0 - qd_peak)/t_plan)*t^2;
                        qd_des = qd_peak + ((0 - qd_peak)/t_plan)*t;
                        qdd_des = (0 - qd_peak)/t_plan;
                    else % we should already be stopped, maintain that.
                        q_des = q_peak;
                        qd_des = zeros(size(q_dot_0));
                        qdd_des = zeros(size(q_0));
                    end
                end
            case 'bernstein'
                % assuming K = [-1, 1] corresponds to final position for now!!
                n_q = length(q_0);
                if ~isnan(k)
                    q1 = P.jrs_info.c_k_bernstein + P.jrs_info.g_k_bernstein.*k;
                    for j = 1:n_q
                        beta{j} = match_deg5_bernstein_coefficients({q_0(j); q_dot_0(j); q_ddot_0(j); q1(j); 0; 0});
                        alpha{j} = bernstein_to_poly(beta{j}, 5);
                    end
                    q_des = zeros(length(q_0), 1);
                    qd_des = zeros(length(q_0), 1);
                    qdd_des = zeros(length(q_0), 1);
                    for j = 1:n_q
                        for coeff_idx = 0:5
                            q_des(j) = q_des(j) + alpha{j}{coeff_idx+1}*t^coeff_idx;
                            if coeff_idx > 0
                                qd_des(j) = qd_des(j) + coeff_idx*alpha{j}{coeff_idx+1}*t^(coeff_idx-1);
                            end
                            if coeff_idx > 1
                                qdd_des(j) = qdd_des(j) + (coeff_idx)*(coeff_idx-1)*alpha{j}{coeff_idx+1}*t^(coeff_idx-2);
                            end
                        end
                    end
                else
                    % bring the trajectory to a stop using previous trajectory...
                    t_plan = P.t_plan;
                    if t <= t_plan && ~all(q_dot_0 == 0) && ~all(q_ddot_0 == 0)
                        % just plug into previous trajectory, but shift time forward by t_plan.
                        [q_des, qd_des, qdd_des] = P.info.desired_trajectory{end}(t + t_plan);
                    else % we should already be stopped, maintain that.
                        q_des = q_0;
                        qd_des = zeros(n_q, 1);
                        qdd_des = zeros(n_q, 1);
                    end
                end
            otherwise
                error('trajectory type not recognized');
            end
        end
        
    end
end

function [q, qd, qdd] = unwrap_traj(res)
    q = res.q_des;
    qd = res.q_dot_des;
    qdd = res.q_ddot_des;
end
