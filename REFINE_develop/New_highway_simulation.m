%% highway_simulation

plot_sim_flag = 1;
plot_AH_flag = 1;
save_result = false; % make it true if you want to save the simulation data

%% set up required objects for simulation
lanewidth = 3.7;
bounds = [0, 2000, -(lanewidth / 2) - 1, ((lanewidth/2) * 5) + 1];
goal_radius = 12;
world_buffer = 1;
t_move = 3;%passing in NewhighwayAGent
t_plan = 3;%change desired idx for this
t_failsafe_move = 3;
verbose_level = 0;
num_ego_vehicles = 1;
num_moving_cars = 15;
num_static_cars = 3;
num_total_cars = num_ego_vehicles + num_moving_cars + num_static_cars;
hlp_lookahead = 90;

worldState = [1 2 3;-3 -1 1];

% DUmmy variables for testing
trajOptProps = rtd.planner.trajopt.TrajOptProps;
trajOptProps.timeForCost = 5.0; % Set the time used for evaluation in the cost function
trajOptProps.planTime = t_plan; % Set the time duration of the nominal plan
trajOptProps.horizonTime = 1.0; % Set the time of the overall trajectory until stop
trajOptProps.doTimeout = false; % Set whether or not to timeout the optimization
trajOptProps.timeoutTime = 0.5; % Set the timeout time for the optimization
trajOptProps.randomInit = false; % Set whether or not to randomize unknown or extra parameters ---> unknown zeros



for j = 1:1000 
    % RESET simulation environment
    World = dynamic_car_world( 'bounds', bounds, ...
        'buffer', world_buffer, 'goal', [1010;3.7], ...
        'verbose', verbose_level, 'goal_radius', goal_radius, ...
        'num_cars', num_total_cars, 'num_moving_cars', num_moving_cars, ...
        't_move_and_failsafe', t_move+t_failsafe_move);

    Agent = highway_cruising_10_state_agent; % takes care of vehicle states and dynamics
    Agent.desired_initial_condition = [10;0; 0; 20;0;0;20;0;0;0];
    Agent.integrator_type= 'ode45';
    
    HLP = simple_highway_HLP; % high-level planner
    HLP.lookahead = hlp_lookahead; 
    AgentHelper = NewhighwayAgentHelper(Agent,HLP,worldState,trajOptProps); % takes care of online planning
    Simulator = rlsimulator(AgentHelper,World,'plot_sim_flag',plot_sim_flag,'plot_AH_flag',plot_AH_flag,'save_result',save_result);

    AgentHelper.S = Simulator;
    Simulator.eval = 1; %turn on evaluation so summary will be saved

    rng(j+1);
    IsDone4 = 0;
    Simulator.epscur = j;
    Simulator.reset();%ORIGINAL
    for i = 1:40
        AgentHelper.planned_path = [linspace(0,1000);repmat([0;0],1,100)];
        [~,~,IsDone,LoggedSignal]=Simulator.step([rand*2-1;rand*2-1]);
        if IsDone == 1 || IsDone == 3 || IsDone == 4 || IsDone == 5
            %crash
            %      crash with safety layer on
            %                      safely stopped but stuck
            %                                           reached goal!
            break
        end
    end
    pause(1)
end

done = 'Simulation Complete';