
clear ; clc; close all;

%% Agent parameters
verbosity = 'DEBUG';

%%% for agent
% agent_urdf = 'fetch_arm_new_dumbbell.urdf';
agent_urdf = 'kinova_without_gripper.urdf';

add_uncertainty_to = 'all'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {'dumbbell_link'}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

% If this flag is true, we use the ArmourCadPatchVisual component instead
visualize_cad = true;

% Add noise to the dynamics
component_options.dynamics.measurement_noise_points = 0;
component_options.dynamics.log_controller = true;
component_options.controller.use_true_params_for_robust = true;

%% Setup the info
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = armour.legacy.load_robot_params(robot, ...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range);
vel_limits = [-1.3963, -1.3963, -1.3963, -1.3963, -1.2218, -1.2218, -1.2218;
                       1.3963,  1.3963,  1.3963,  1.3963,  1.2218,  1.2218,  1.2218]; % matlab doesn't import these from urdf so hard code into class
input_limits = [-56.7, -56.7, -56.7, -56.7, -29.4, -29.4, -29.4;
                       56.7,  56.7,  56.7,  56.7,  29.4,  29.4,  29.4]; % matlab doesn't import these from urdf so hard code into class
transmision_inertia = [8.02999999999999936 11.99620246153036440 9.00254278617515169 11.58064393167063599 8.46650409179141228 8.85370693737424297 8.85873036646853151]; % matlab doesn't import these from urdf so hard code into class
M_min_eigenvalue = 5.095620491878957; % matlab doesn't import these from urdf so hard code into class

agent_info = armour.agent.ArmourAgentInfo(robot, params, ...
                joint_velocity_limits=vel_limits, ...
                joint_torque_limits=input_limits, ...
                transmission_inertia=transmision_inertia, ...
                M_min_eigenvalue=M_min_eigenvalue);

% Visualization?
visual = [];
if visualize_cad
    % this could be improved
    visual = armour.agent.ArmourCadPatchVisual(armour.agent.ArmourAgentInfo.empty(),armour.agent.ArmourAgentState.empty(),armour.agent.ArmKinematics.empty());
    camlight
end

controller = [];
% controller = 'armour.agent.ArmourMexController';

% Create
agent = armour.ArmourAgent(agent_info, visual=visual, ...
                component_options=component_options, ...
                component_logLevelOverride=verbosity, ...
                controller=controller);

%% Demo section of copy-ability
% A.visual.plot()
% axis equal
% 
% % Demo the ability to copy paste
% opts = A.getoptions
% A_2 = ArmourAgent.from_options(robot, params, opts)
% 
% % Change out the visual for the third copy paste
% opts.components.visual = 'ArmourCadPatchVisual';
% opts.component_options.visual = ArmourCadPatchVisual.defaultoptions();
% % Also name it
% opts.name = 'CAD'
% 
% % Demo
% A_3 = ArmourAgent.from_options(robot, params, opts)
% figure; view(3); grid on
% plot(A_3.visual)
% axis equal; camlight
% disp("press enter to continue")
% pause

%% Create the simulation
sim = armour.ArmourSimulation;
sim.setup(agent)
sim.initialize()
% not the proper way to do it, but proper way isn't implemented yet
sim.visual_system.enable_camlight = visualize_cad;
sim.visual_system.redraw();
%sim.run(max_steps=1);

disp("press enter to continue")
pause

%% Interface of Planner should improve
trajOptProps = rtd.planner.trajopt.TrajOptProps;
trajOptProps.planTime = 0.5;
trajOptProps.horizonTime = 1.0;
trajOptProps.doTimeout = false;
trajOptProps.timeoutTime = 0.5;
trajOptProps.randomInit = true;
trajOptProps.timeForCost = 1.0;

input_constraints_flag = false;
use_robust_input = false;
smooth_obs = false;

planner = armour.ArmourPlanner( ...
        trajOptProps, sim.agent, ...
        input_constraints_flag=input_constraints_flag,...
        use_robust_input=use_robust_input,...
        smooth_obs=smooth_obs, ...
        traj_type="bernstein", ...
        verboseLevel='DEBUG');

%% HLP stuff to migrate
HLP = robot_arm_straight_line_HLP();
world_info.goal = sim.goal_system.goal_position;
agent_info = struct;
agent_info.n_states = sim.agent.state.n_states;
agent_info.n_inputs = sim.agent.controller.n_inputs;
agent_info.n_links_and_joints = sim.agent.info.n_links_and_joints;
agent_info.joint_state_limits = [sim.agent.info.joints.position_limits];
agent_info.joint_speed_limits = [sim.agent.info.joints.velocity_limits];
agent_info.joint_input_limits = [sim.agent.info.joints.torque_limits];
agent_info.joint_state_indices = sim.agent.state.position_indices;
agent_info.joint_speed_indices = sim.agent.state.velocity_indices;
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

HLP.setup(agent_info,world_info);
HLP.make_new_graph_every_iteration_flag = 1;
HLP.sampling_timeout = 0.5;

%% Run planning step by step
lookahead = 0.4;
iter = 0;
pausing = false;
% while true
%     if pausing
%         pause
%     end
%     iter = iter + 1;
%     sim.run(max_steps=1);
%     pause(0.1)
% end

cb = @(sim) planner_callback(sim, planner, agent_info, world_info, lookahead, HLP);
sim.run(max_steps=100, pre_step_callback={cb});

function info = planner_callback(sim, planner, agent_info, world_info, lookahead, HLP)
    % Get the end state
    time = sim.agent.state.time(end);
    ref_state = sim.agent.controller.trajectories{end}.getCommand(time);
    agent_info.state = sim.agent.state.state(:,end);

    q_des = HLP.get_waypoint(agent_info,world_info,lookahead);
    if isempty(q_des)
        disp('Waypoint creation failed! Using global goal instead.')
        q_des = HLP.goal ;
    end

    % get the sensor readings at the time
    worldState.obstacles = rtd.sim.sensors.zonotope_sensor(sim.world, sim.agent, time);
    [trajectory, plan_info] = planner.planTrajectory(ref_state, worldState, q_des);
    %FO = plan_info.rsInstances{2}.FO;
    %jrsinfo = plan_info.rsInstances{1}.jrs_info;
    
    if ~isempty(trajectory)
        sim.agent.controller.setTrajectory(trajectory)
    end

    info = plan_info;
end