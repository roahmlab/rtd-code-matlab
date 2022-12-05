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
num_planners = 10;
allow_replan_errors = true ;
first_iter_pause_flag = false;
use_q_plan_for_cost = false; % otherwise use q_stop (q at final time)
input_constraints_flag = false;
save_FO_zono_flag = true;
traj_type = 'bernstein'; %'orig' or 'bernstein'
% For bernstein, due to precision errors in optimization, this actually needs to be greater!
% 1e-8 works for orig, 2e-5 for bernstein.
comparison_delta = 2e-5;
random_init = false; % Random init results in less similar optimizations

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

P_func = @()uarmtd_planner_wrapped_comparison(...
    'agent', A, ...
    'verbose', verbosity, ...
    'traj_type', traj_type, ...
    'random_init', random_init, ...
    'comparison_delta', comparison_delta, ...
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
P = cell(1,num_planners);
for i=1:num_planners
    P{i} = P_func();
end
S = simulator_armtd_ensemble(A,W,P,'allow_replan_errors',allow_replan_errors,'max_sim_time',1000,'max_sim_iterations',1000) ;
S.stop_sim_when_ultimate_bound_exceeded = false;
% create a parpool for the simulator ensemble
delete(gcp('nocreate'))
parpool("Processes", num_planners)


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

%% Get the range and variance of each of the planner k_opts
% mean, median, stddev, low, high
summary.k_opts_stats
summary.k_opts_new_stats
disp("Press enter to continue")
%pause

%% Test setup
%num_workers = 120; % specify 0 to run without parallel pool
num_trials = 500;
timeout = 36000;
disp("Proceeding to auto-test")

% delete(gcp('nocreate'))
% if num_workers > 0
%     parpool("Processes", num_workers)
% end


%% Auto test
% Create num_workers and num_trials worlds
old_time = cell(1, num_trials);
new_time = cell(1, num_trials);
errored = cell(1, num_trials);
summaries = cell(1, num_trials);
timeouts = zeros(1, num_trials);
clear summary
% note that i runs in nondeterministic manner regardless!
subdiv = 10;
subdiv_amount = num_trials/subdiv;
max_eps_q = 0;
max_eps_qd = 0;
for k=0:subdiv-1
bad_ids = [];
%parfor (i = k*subdiv_amount+1:(k+1)*subdiv_amount, num_workers)
for i = k*subdiv_amount+1:(k+1)*subdiv_amount
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

    W = fetch_base_world_static('include_base_obstacle', true, 'goal_radius', pi/30, 'N_random_obstacles', N_random_obstacles,'dimension',dimension,'workspace_goal_check', 0,...
        'verbose',verbosity, 'creation_buffer', 0.05, 'base_creation_buffer', 0.025) ;

    % Create new planner
%     P = uarmtd_planner_wrapped_comparison(...
%         'agent', A, ...
%         'wait_on_first_run', false, ...
%         'verbose', verbosity, ...
%         'traj_type', traj_type, ...
%         'random_init', random_init, ...
%         'comparison_delta', comparison_delta, ...
%         'first_iter_pause_flag', first_iter_pause_flag, ...
%         'use_q_plan_for_cost', use_q_plan_for_cost, ...
%         'input_constraints_flag', input_constraints_flag, ...
%         'save_FO_zono_flag', save_FO_zono_flag) ;

    P = cell(1,num_planners);
    for m=1:num_planners
        P{m} = P_func();
    end
    
    % update world and reset arm
    I = A.get_agent_info ;
    W.setup(I)
    % place arm at starting configuration
    A.state(A.joint_state_indices) = W.start ;

    % create simulator
    S = simulator_armtd_ensemble(A,W,P,'allow_replan_errors',allow_replan_errors,'max_sim_time',timeout,'max_sim_iterations',1000) ;
    S.stop_sim_when_ultimate_bound_exceeded = false;
    
    % run the world
    summary = S.run()
    
    error_count = 0;
    for m=1:num_planners
        error_count = error_count + sum(P{m}.info.error_count);
        max_eps_q = max(P{m}.max_eps_q, max_eps_q);
        max_eps_qd = max(P{m}.max_eps_qd, max_eps_qd);
    end

    if (error_count > 0)
        errored(i) = {P};
        bad_ids = [bad_ids, i];
    end
    %old_time(i) = {P.info.planning_time};
    %new_time(i) = {P.new_info.planning_time};
    summaries(i) = {summary};
    timeouts(i) = summary.goal_check;
end
if (max_eps_q > 0) || (max_eps_qd > 0)
    max_eps_q
    max_eps_qd
    bad_ids
end
end

%% stats new
% mean median std min max range
for i=1:num_trials
    overall_range = zeros(size(summaries{i}.k_opts_stats{1}(:, 6)));
    for j=1:length(summaries{i}.k_opts_stats)
        if any(isnan(summaries{i}.k_opts_stats{j}(:, 3)))
            continue
        end
        overall_range = max(overall_range, summaries{i}.k_opts_stats{j}(:, 3));
    end
    i, overall_range
    pause
end

%% stats old
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

% get the mask of successful trials
success_mask = cellfun(@isempty, errored);

% disp
disp("Error count:")
sum(~success_mask)

disp("Timeouts:")
sum(timeouts)

disp("Old planner time stats (success)[mu,sigma,med]:")
stats = stats_out(old_time, success_mask)

disp("New planner time stats (success)[mu,sigma,med]:")
stats = stats_out(new_time, success_mask)

disp("Old planner time stats (fails)[mu,sigma,med]:")
stats = stats_out(old_time, ~success_mask)

disp("New planner time stats (fails)[mu,sigma,med]:")
stats = stats_out(new_time, ~success_mask)


%% Find max epsilon
problems = errored(~success_mask);

%Store
max_eps_q = 0;
max_eps_qd = 0;
for i=1:length(problems)

    % old_fun = problems{i}.info.desired_trajectory;
    % new_fun = problems{i}.new_info.desired_trajectory;
    % num_fun = length(old_fun);

    % T = 0:problems{i}.time_discretization:problems{i}.t_stop/2;
    % for j=1:num_fun
    %     for k=1:length(T)
    %         T(k)
    %         [q_tmp, qd_tmp, ~] = old_fun{j}(T(k));
    %         [q_tmp_comp, qd_tmp_comp, ~] = new_fun{j+1}(T(k));
    
    %         max_eps_q = max(norm(q_tmp-q_tmp_comp), max_eps_q);
    %         max_eps_qd = max(norm(qd_tmp-qd_tmp_comp), max_eps_qd);
    %     end
    % end
    % i
    max_eps_q = max(problems{i}.max_eps_q, max_eps_q);
    max_eps_qd = max(problems{i}.max_eps_qd, max_eps_qd);
end
max_eps_q
max_eps_qd


%% Extra stuff written for validation
id = 62;
full_q_old = [];
full_q_new = [];
full_qd_old = [];
full_qd_new = [];
for i=1:length(errored{id}.info.traj_q)
    full_q_old = [full_q_old, errored{id}.info.traj_q{i}(:,1:50)];
    full_q_new = [full_q_new, errored{id}.new_info.traj_q{i}(:,1:50)];
    full_qd_old = [full_qd_old, errored{id}.info.traj_qd{i}(:,1:50)];
    full_qd_new = [full_qd_new, errored{id}.new_info.traj_qd{i}(:,1:50)];
end
T=0:errored{id}.time_discretization:errored{id}.time_discretization*(length(full_q_old)-1);

for i=1:length(errored{id}.info.k_opt)
    [i, norm(errored{id}.info.k_opt{i} - errored{id}.new_info.k_opt{i})]
end

plot(T, full_q_old.'-full_q_new.')
hold on
plot(T, full_qd_old.'-full_qd_new.')
plot(T, full_q_old.')
plot(T, full_q_new.')
plot(T, full_qd_new.')
plot(T, full_qd_old.')
hold off

k_exists_int = cellfun(@isnan, errored{id}.info.k_opt, 'UniformOutput', false);
k_exists = cellfun(@sum, k_exists_int);
T_k = 0:errored{id}.t_plan:T(end);
% Make it a solid step
k_exists_step = [k_exists; k_exists];
k_exists_step = k_exists_step(:).';
T_k_step = [0:errored{id}.t_plan:T(end); ...
            (0:errored{id}.t_plan:T(end)) + errored{id}.t_plan];
T_k_step = T_k_step(:).';

% Check
plot(T, full_qd_new.')
hold on
plot(T, full_qd_old.')
plot(T_k_step, k_exists_step)
hold off

%% Extra Func
function [size, res] = stats_out(times, success_mask)
    temp = cell2mat(times(success_mask));
    mu = mean(temp);
    sigma = std(temp);
    med = median(temp);
    res = [mu, sigma, med];
end
