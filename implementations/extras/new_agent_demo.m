
clear ; clc ; figure(1); clf; view(3); grid on;

%% Agent parameters
verbosity = LogLevel.DEBUG;

%%% for agent
agent_urdf = 'fetch_arm_new_dumbbell.urdf';

add_uncertainty_to = 'link'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {'dumbbell_link'}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];

% If this flag is true, we use the ArmourCadPatchVisual component instead
visualize_cad = false;

% Add noise to the dynamics
component_options.dynamics.measurement_noise_points = 11;
component_options.controller.use_true_params_for_robust = true;

%% Setup the info
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = load_robot_params(robot, ...
                           'add_uncertainty_to', add_uncertainty_to, ...
                           'links_with_uncertainty', links_with_uncertainty,...
                           'uncertain_mass_range', uncertain_mass_range);
vel_limits = [-1.256, -1.454, -1.571, -1.521, -1.571, -2.268, -2.268;
               1.256,  1.454,  1.571,  1.521,  1.571,  2.268,  2.268]; % matlab doesn't import these from urdf
input_limits = [-33.82, -131.76, -76.94, -66.18, -29.35, -25.70, -7.36;
                 33.82,  131.76,  76.94,  66.18,  29.35,  25.70,  7.36]; % matlab doesn't import these from urdf
M_min_eigenvalue = 0.0017;

agent_info = ArmourAgentInfo(robot, params, ...
                joint_velocity_limits=vel_limits, ...
                joint_torque_limits=input_limits, ...
                M_min_eigenvalue=M_min_eigenvalue);

%% Visualization?
visual = [];
if visualize_cad
    % this could be improved
    visual = ArmourCadPatchVisual(ArmourAgentInfo.empty(),ArmourAgentState.empty(),ArmKinematics.empty());
    camlight
end

%% Create
A = ArmourAgent(agent_info, visual=visual, ...
                component_options=component_options, ...
                component_logLevelOverride=verbosity);
A.visual.plot()
axis equal

% Demo the ability to copy paste
opts = A.getoptions
A_2 = ArmourAgent.from_options(robot, params, opts)

% Change out the visual for the third copy paste
opts.components.visual = 'ArmourCadPatchVisual';
opts.component_options.visual = ArmourCadPatchVisual.defaultoptions();
% Also name it
opts.name = 'CAD'

% Demo
A_3 = ArmourAgent.from_options(robot, params, opts)
figure; view(3); grid on
plot(A_3.visual)
axis equal; camlight
disp("press enter to continue")
pause

%% Create the simulation
sim = ArmourSimulation
sim.setup(A_2)
sim.initialize()
%sim.run(max_steps=1);

disp("press enter to continue")
pause

%% Explicit here for now

trajOptProps = TrajOptProps;
trajOptProps.planTime = 0.5;
trajOptProps.horizonTime = 1.0;
trajOptProps.doTimeout = false;
trajOptProps.timeoutTime = 0.5;
trajOptProps.randomInit = true;
trajOptProps.timeForCost = 0.5;

input_constraints_flag = false;
use_robust_input = false;
smooth_obs = false;

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

planner = ArmourPlanner( ...
        trajOptProps, sim.agent, ...
        input_constraints_flag, use_robust_input, smooth_obs, 'orig' ...
    );

worldState = WorldState;
worldState.obstacles = num2cell(sim.obstacles);

%% Run planning step by step
lookahead = 1;
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

cb = @(sim) planner_callback(sim, planner, agent_info, world_info, lookahead, HLP, worldState);
sim.run(max_steps=20, pre_step_callback={cb});

function info = planner_callback(sim, planner, agent_info, world_info, lookahead, HLP, worldState)
    world = struct2array(sim.world);
    mask = ~strcmp({world.uuid}, sim.agent.uuid);
    representations = [world(mask).representation];
    obstacles = arrayfun(@(x)x.get_zonotope,representations);

    time = sim.agent.state.time(end);
    ref_state = sim.agent.controller.trajectories{end}.getCommand(time);
    agent_info.state = sim.agent.state.state(:,end);
    
    q_des = HLP.get_waypoint(agent_info,world_info,lookahead) ;
    if isempty(q_des)
        disp('Waypoint creation failed! Using global goal instead.')
        q_des = HLP.goal ;
    end

    [trajectory, plan_info] = planner.planTrajectory(ref_state, worldState, q_des);
    FO = plan_info.rsInstances{2}.FO;
    jrsinfo = plan_info.rsInstances{1}.jrs_info;
    
    try
        % Update k_opt for info struct
        k_opt = trajectory.getTrajParams();
        disp('New trajectory found!');
        sim.agent.controller.setTrajectory(trajectory);
        trajopt_failed = false;
    catch
        % If invalid trajectory, just make k_opt nan, and don't
        % update the latest trajectory.
        disp('Unable to find new trajectory!')
        k_opt = nan;
        trajopt_failed = true;
    end

    info = struct;
end