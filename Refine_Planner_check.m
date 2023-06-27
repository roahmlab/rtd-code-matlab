

%refine_planner check
reachsets = loaders.RSLoader('converted_Au_frs.h5');
collection_group = reachsets.getGroup(0);
terminal_group = collection_group.getGroup(0);
first_search = reachsets.getSearchSet.getZonos;
rs = FRS_loader_speed_change_test2('converted_Au_frs.h5');
for i = 1: length(rs.robotState)
    for j = 1:length(rs.robotState{i})
        if(i<=21)
            state = rs.robotState{i}{j};
            vehrs_ = rs.vehrs;
            first_search = reachsets.getSearchSet.getZonos;
            t_move = 3;
            t_plan = 6;
            t_failsafe_move = 3;
            verbose_level = 0;
            num_ego_vehicles = 1;
            num_moving_cars = 15;
            num_static_cars = 3;
            for i =1:length(first_search)
                if(t_plan == sum(first_search{i}.Z))
                    desired_idx_ = i; %at t_plan
                end
            end
            
            HLP = simple_highway_HLP();
            trajOptProps = rtd.planner.trajopt.TrajOptProps;
             % DUmmy variables for testing
            trajOptProps.timeForCost = 1.0; % Set the time used for evaluation in the cost function
            trajOptProps.planTime = 0.5; % Set the time duration of the nominal plan
            trajOptProps.horizonTime = 1.0; % Set the time of the overall trajectory until stop
            trajOptProps.doTimeout = false; % Set whether or not to timeout the optimization
            trajOptProps.timeoutTime = 0.5; % Set the timeout time for the optimization
            trajOptProps.randomInit = false; % Set whether or not to randomize unknown or extra parameters
            
            dummy_start_line = 3;  % Example value for the start_line argument
            dummy_bounds = [dummy_start_line -dummy_start_line 0 12];  % Example value for the bounds argument
            dummy_N_obstacles = 2;  % Example value for the N_obstacles argument
            
            W = dynamic_car_world('start_line', dummy_start_line, 'bounds', dummy_bounds, 'N_obstacles', dummy_N_obstacles);
            Agent = highway_cruising_10_state_agent; % takes care of vehicle states and dynamics
            Agent.desired_initial_condition = [10;0; 0; 20;0;0;20;0;0;0];
            Agent.integrator_type= 'ode45';
            HLP = simple_highway_HLP; % high-level planner
            HLP.lookahead = 90; 
            AH = highwayAgentHelper(Agent,rs,HLP,'t_plan',t_plan,'t_move',t_move,'t_failsafe_move',t_failsafe_move,...
            'verbose',verbose_level); % takes care of online planning
          
            agent_info = AH.get_agent_info();
            worldInfo = W.get_world_info(agent_info);
            % [waypoint,lane_idx_] = HLP.get_waypoint(worldInfo,state);
            
            %dummy variables for testing
            waypoint1 = [10, 20];  % Example waypoint 1 [x1, y1]
            waypoint2 = [15, 25];  % Example waypoint 2 [x2, y2]
            waypoint3 = [20, 30];  % Example waypoint 3 [x3, y3]
            
            % Store waypoints in a cell array/dummy variables
            waypoint = {waypoint1, waypoint2, waypoint3};
            
            options = struct();
            options.input_constraints_flag = true;%where is input getting passed from/why is it used?
            options.use_robust_input = true;%why do we need a robust input?
            options.manu_type = 'speed change';
            options.verboseLevel = 'INFO';

            plan = Refine_Planner_test2 (vehrs_,rs,state,desired_idx_,waypoint,trajOptProps);

            disp('end of refine planner check')
        end
    end
end