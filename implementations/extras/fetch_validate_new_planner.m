%% description
% This script generates an agent, world, and planner. It plugs them in to
% the simulator framework, and then runs a single simulation.
%
% Authors: Shreyas Kousik and Patrick Holmes
% Created: 29 July 2019
% Updated: 3 August 2019
% Updated: 19 August 2019 - moving RTD to 3D and using a 1 link model
% Updated: 22 August 2019 - moving RTD to 3D and using Fetch model
% Updated: 18 September 2019 - using rotatotope RTD planner now
% Updated: Continually between 18 September 2019 and 18 March 2020
% Updated: 4 August 2020 - update to new code using `robot_rotatotope_RTD_planner`
% Updated: 21 September 2021 - update for new armtd_dev repo
% Updated: 30 November 2021 (and before) - update to test input constraints
% Updated: 26 April 2022 (and before) - update to new `uarmtd_planner` and `uarmtd_agent`

clear ; clc ; figure(1); clf; view(3); grid on;

%% user parameters
N_random_obstacles = 10; % don't use more than 40!! there's a limit in the CUDA code 
dimension = 3 ;
verbosity = 10 ;

%%% for planner
allow_replan_errors = true ;
first_iter_pause_flag = true;
use_q_plan_for_cost = false; % otherwise use q_stop (q at final time)
input_constraints_flag = false;
save_FO_zono_flag = true;

%%% for agent
% agent_urdf = 'fetch_arm_7DOF.urdf';
agent_urdf = 'fetch_arm_new_dumbbell.urdf';
% agent_urdf = 'fetch_og_arm_reduced_7dof.urdf';

add_uncertainty_to = 'link'; % choose 'all', 'link', or 'none'
% links_with_uncertainty = {}; % if add_uncertainty_to = 'link', specify links here.
links_with_uncertainty = {'dumbbell_link'}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

agent_move_mode = 'integrator' ; % pick 'direct' or 'integrator'
use_CAD_flag = false;
add_measurement_noise_ = false;
measurement_noise_size_ = 11;

%%% for LLC
use_true_params_for_robust = true;

%% robot params:
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot, ...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range);
joint_speed_limits = [-1.256, -1.454, -1.571, -1.521, -1.571, -2.268, -2.268;
                       1.256,  1.454,  1.571,  1.521,  1.571,  2.268,  2.268]; % matlab doesn't import these from urdf
joint_input_limits = [-33.82, -131.76, -76.94, -66.18, -29.35, -25.70, -7.36;
                       33.82,  131.76,  76.94,  66.18,  29.35,  25.70,  7.36]; % matlab doesn't import these from urdf
M_min_eigenvalue = 0.0017;

% % without dumbbell
% agent_urdf = 'fetch_arm_7DOF.urdf';
% robot = importrobot(agent_urdf);
% robot.DataFormat = 'col';
% robot.Gravity = [0 0 -9.81];
% params = load_robot_params(robot, 'add_uncertainty_to', 'all');
% joint_speed_limits = [-1.256, -1.454, -1.571, -1.521, -1.571, -2.268, -2.268;
%                        1.256,  1.454,  1.571,  1.521,  1.571,  2.268,  2.268]; % matlab doesn't import these from urdf
% joint_input_limits = [-33.82, -131.76, -76.94, -66.18, -29.35, -25.70, -7.36;
%                        33.82,  131.76,  76.94,  66.18,  29.35,  25.70,  7.36]; % matlab doesn't import these from urdf

%% automated from here!
% agent
A = uarmtd_agent(robot, params,...
                 'verbose', verbosity,...
                 'animation_set_axes_flag', 0,... 
                 'animation_set_view_flag', 0,...
                 'move_mode', agent_move_mode,...
                 'use_CAD_flag', use_CAD_flag,...
                 'joint_speed_limits', joint_speed_limits, ...
                 'joint_input_limits', joint_input_limits, ...
                 'add_measurement_noise_', add_measurement_noise_, ...
                 'measurement_noise_size_', measurement_noise_size_,...
                 'M_min_eigenvalue', M_min_eigenvalue);

% LLC
A.LLC = uarmtd_robust_CBF_LLC('verbose', verbosity, ...
                              'use_true_params_for_robust', use_true_params_for_robust);
% A.LLC = uarmtd_robust_CBF_MEX_LLC('verbose', verbosity, ...
%                               'use_true_params_for_robust', use_true_params_for_robust);
A.LLC.setup(A);

%% world
W = fetch_base_world_static('include_base_obstacle', true, 'goal_radius', pi/30, 'N_random_obstacles', N_random_obstacles,'dimension',dimension,'workspace_goal_check', 0,...
    'verbose',verbosity, 'creation_buffer', 0.05, 'base_creation_buffer', 0.025) ;

%% planner
%%% HIGH LEVEL PLANNER PARAMS (STILL NEEDS UPDATING) %%%
%     HLP_timeout = 2 ; 
%     HLP_grow_tree_mode = 'new' ;
%     plot_while_sampling_flag = false ;
%     make_new_graph_every_iteration = false ;
%     plot_HLP_flag = true ; % for planner
%     plot_waypoint_flag = true ; % for HLP
%     plot_waypoint_arm_flag  = true ; % for HLP
%     lookahead_distance = 0.1 ;

P = uarmtd_planner_wrapped_comparison(...
    'agent', A, ...
    'verbose', verbosity, ...
    'random_init', false, ... % Random init results in less similar optimizations
    'first_iter_pause_flag', first_iter_pause_flag, ...
    'use_q_plan_for_cost', use_q_plan_for_cost, ...
    'input_constraints_flag', input_constraints_flag, ...
    'save_FO_zono_flag', save_FO_zono_flag) ;


%% set up world using arm
I = A.get_agent_info ;
W.setup(I)

% place arm at starting configuration:
% W.start = zeros(A.n_states/2, 1); % put in "home" config
% W.start = [0.8375; 0.5705; 0.6680; 1.2783; 2.6335; 0.9658; -1.0100];
% A.state(A.joint_state_indices) = W.start ;

% create simulator
S = simulator_armtd(A,W,P,'allow_replan_errors',allow_replan_errors,'max_sim_time',1000,'max_sim_iterations',1000) ;
S.stop_sim_when_ultimate_bound_exceeded = false;

% create .csv file:
% write_fetch_scene_to_csv(W);


%% run simulation
summary = S.run()

%% plotting
figure(1) ; clf ; axis equal ; hold on ; grid on

plot(W)

if dimension == 3
    view(3)
end

animate(A)

%% Check if any errors occured
if (sum(P.info.error_count) > 0)
    disp("Disparity occured!")
    disp("Set breakpoint and check P.info and P.new_info!")
    pause;
else
    % Save the time calculation
    to_graph.old_time = P.info.planning_time;
    to_graph.new_time = P.new_info.planning_time;
end

disp("Proceeding to auto-test")

%% Auto test
% Create 100 worlds
old_time = [];
new_time = [];
errored = [];
for i = 1:100
    W = fetch_base_world_static('include_base_obstacle', true, 'goal_radius', pi/30, 'N_random_obstacles', N_random_obstacles,'dimension',dimension,'workspace_goal_check', 0,...
        'verbose',verbosity, 'creation_buffer', 0.05, 'base_creation_buffer', 0.025) ;

    % Create new planner
    P = uarmtd_planner_wrapped_comparison(...
        'agent', A, ...
        'wait_on_first_run', false, ...
        'verbose', verbosity, ...
        'first_iter_pause_flag', first_iter_pause_flag, ...
        'use_q_plan_for_cost', use_q_plan_for_cost, ...
        'input_constraints_flag', input_constraints_flag, ...
        'save_FO_zono_flag', save_FO_zono_flag) ;
    
    % update world and reset arm
    I = A.get_agent_info ;
    W.setup(I)
    % place arm at starting configuration
    A.state(A.joint_state_indices) = W.start ;

    % create simulator
    S = simulator_armtd(A,W,P,'allow_replan_errors',allow_replan_errors,'max_sim_time',3600,'max_sim_iterations',1000) ;
    S.stop_sim_when_ultimate_bound_exceeded = false;
    
    % run the world
    S.run()
    
    if (sum(P.info.error_count) > 0)
        errored = [errored, P];
    end
    old_time = [old_time, P.info.planning_time];
    new_time = [new_time, P.new_info.planning_time];
end

%% stats

% Graph
oldcumtime = cumsum([0; to_graph.old_time(:)]);
newcumtime = cumsum([0; to_graph.new_time(:)]);
x = 0:length(to_graph.old_time);
figure(2); clf(2); hold on
plot(x,oldcumtime)
plot(x,newcumtime)
hold off
legend(["old time", "new time"],'Location','northwest');
xlabel("Planning Iteration")
ylabel("Cumulative Time (s)")

% disp
disp("Error count:")
length(errored)
disp("Old planner time stats:")
mu = mean(old_time)
sigma = std(old_time)
med = median(old_time)
disp("New planner time stats:")
mu = mean(new_time)
sigma = std(new_time)
med = median(new_time)