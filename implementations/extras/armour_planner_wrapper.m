classdef armour_planner_wrapper < robot_arm_generic_planner
    
    
    properties
        % processed
        agent
        planner
        latest_trajectory
        
        % UNPROCESSED
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
        function P = armour_planner_wrapper(varargin)
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
            trajOptProps.timeout = P.t_move;
            trajOptProps.horizon = P.t_stop;
            if P.use_q_plan_for_cost
                timeForCost = P.t_move;
            else
                timeForCost = P.t_stop;
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
                    trajOptProps, timeForCost, robotInfo, worldInfo, ...
                    input_constraints_flag, use_robust_input, smooth_obs, P.traj_type ...
                );
        end
        
        
        function init_info(P)
            P.info = struct('T',[],'U',[],'Z',[],'waypoint',[],...
                'obstacles',[],'q_0',[],'q_dot_0',[],'k_opt',[],...
                'desired_trajectory', [], 't_move', [], ...
                'FO_zono', [], 'sliced_FO_zono', []) ;
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
                P.vdisp('Press Enter to Continue:',6)
                pause; 
            end
            P.iter = P.iter + 1;
            planning_time = tic;
            q_des = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
            if isempty(q_des)
                P.vdisp('Waypoint creation failed! Using global goal instead.', 3)
                q_des = P.HLP.goal ;
            end
                        
            % get current obstacles and create constraints
            P.vdisp('Running Wrapped Planner!',6)
            worldState = WorldState;
            worldState.obstacles = world_info.obstacles;
            
            robotState = ArmRobotState;
            robotState.time = agent_info.time(end);
            robotState.q = q_0;
            robotState.q_dot = q_dot_0;
            robotState.q_ddot = q_ddot_0;
            
            waypoint = q_des;
            
            trajectory = P.planner.planTrajectory(robotState, worldState, waypoint);
            
            % process result            
            %if ~trajectory.validate()
            %    P.vdisp('New trajectory found!',3);
            %else % no safe trajectory parameter found:
            %    P.vdisp('Unable to find new trajectory!',3)
            %    k_opt = nan;
            %end
            toc(planning_time);
            
            try
                % Update k_opt for info struct
                k_opt = trajectory.getTrajParams();
                P.vdisp('New trajectory found!',3);
                P.latest_trajectory = trajectory;
            catch
                % If invalid trajectory, just make k_opt nan, and don't
                % update the latest trajectory.
                P.vdisp('Unable to find new trajectory!',3)
                k_opt = nan;
            end
            
            % save info
            P.info.desired_trajectory = [P.info.desired_trajectory, {@(t) unwrap_traj(P.latest_trajectory.getCommand(robotState.time + t))}];
            P.info.t_move = [P.info.t_move, {P.t_move}];
            P.info.waypoint = [P.info.waypoint, {q_des}] ;
            P.info.obstacles = [P.info.obstacles, {world_info.obstacles}] ;
            P.info.q_0 = [P.info.q_0, {q_0}] ;
            P.info.q_dot_0 = [P.info.q_dot_0, {q_dot_0}] ;
            P.info.k_opt = [P.info.k_opt, {k_opt}] ;
            %if P.save_FO_zono_flag
            %    for i = 1:P.jrs_info.n_t
            %        for j = 1:P.agent_info.params.pz_nominal.num_bodies
            %            FO_zono{i}{j} = zonotope(FO{i}{j});
            %            if trajopt_failed
            %                % no safe slice
            %                sliced_FO_zono{i}{j} = [];
            %            else
            %                % slice and save
            %                fully_sliceable_tmp = polyZonotope_ROAHM(FO{i}{j}.c, FO{i}{j}.G, [], FO{i}{j}.expMat, FO{i}{j}.id);
            %                sliced_FO_zono{i}{j} = zonotope([slice(fully_sliceable_tmp, k_opt), FO{i}{j}.Grest]);
            %            end
            %        end
            %    end
            %    P.info.FO_zono = [P.info.FO_zono, {FO_zono}];
            %    P.info.sliced_FO_zono = [P.info.sliced_FO_zono, {sliced_FO_zono}];
            %end

            % create outputs:
            T = 0:P.time_discretization:P.t_stop ;
            U = zeros(agent_info.n_inputs, length(T));
            Z = zeros(agent_info.n_states, length(T));
            for i = 1:length(T)
                % NOTE THAT THIS USES THE ABOVE GENERATED INFO THINGY
                [q_tmp, qd_tmp, ~] = P.info.desired_trajectory{end}(T(i));
                Z(agent_info.joint_state_indices, i) = q_tmp;
                Z(agent_info.joint_speed_indices, i) = qd_tmp;
            end
            info = P.info;
        end
    end
end

function [q, qd, qdd] = unwrap_traj(res)
    q = res.q_des;
    qd = res.q_dot_des;
    qdd = res.q_ddot_des;
end