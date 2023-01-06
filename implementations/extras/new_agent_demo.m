
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

%% Explicit here for now

trajOptProps = TrajOptProps;
trajOptProps.planTime = 0.5;
trajOptProps.horizonTime = 1.0;
trajOptProps.doTimeout = false;
trajOptProps.timeoutTime = 0.5;
trajOptProps.randomInit = true;
trajOptProps.timeForCost = 0.5;

robotInfo = ArmRobotInfo;
robotInfo.params = sim.agent.params;
robotInfo.link_poly_zonotopes = {sim.agent.info.links.poly_zonotope};
robotInfo.LLC_info = {};
robotInfo.LLC_info.ultimate_bound = sim.agent.controller.ultimate_bound;
robotInfo.LLC_info.ultimate_bound_position = sim.agent.controller.ultimate_bound_position;
robotInfo.LLC_info.ultimate_bound_velocity = sim.agent.controller.ultimate_bound_velocity;
robotInfo.LLC_info.alpha_constant = sim.agent.controller.alpha_constant;
robotInfo.joint_input_limits = [sim.agent.info.joints.torque_limits];

worldInfo = WorldInfo;

input_constraints_flag = false;
use_robust_input = false;
smooth_obs = false;

planner = ArmourPlanner( ...
        trajOptProps, robotInfo, worldInfo, ...
        input_constraints_flag, use_robust_input, smooth_obs, 'orig' ...
    );

%% ignore the below
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
        P.vdisp('Generating initial trajectory', 6)
        robotState = ArmRobotState;
        robotState.time = 0.0;
        robotState.q = q_0;
        robotState.q_dot = q_dot_0;
        robotState.q_ddot = q_ddot_0;
        rs = P.planner.jrsHandle.getReachableSet(robotState, false);
        traj = P.planner.trajectoryFactory(robotState,{rs});
        traj.setTrajectory(zeros(traj.param_shape,1));
        P.info.desired_trajectory = [P.info.desired_trajectory, {@(t) unwrap_traj(traj.getCommand(t))}];
        P.latest_trajectory = traj;

        if P.wait_on_first_run
            P.vdisp('Press Enter to Continue:')
            pause;
        end
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

    [trajectory, plan_info] = P.planner.planTrajectory(robotState, worldState, waypoint);
    FO = plan_info.rsInstances{2}.FO;
    jrsinfo = plan_info.rsInstances{1}.jrs_info;

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
        trajopt_failed = false;
    catch
        % If invalid trajectory, just make k_opt nan, and don't
        % update the latest trajectory.
        P.vdisp('Unable to find new trajectory!',3)
        k_opt = nan;
        trajopt_failed = true;
    end

    % save info
    P.info.desired_trajectory = [P.info.desired_trajectory, {@(t) unwrap_traj(P.latest_trajectory.getCommand(robotState.time + t))}];
    P.info.t_move = [P.info.t_move, {P.t_move}];
    P.info.waypoint = [P.info.waypoint, {q_des}] ;
    P.info.obstacles = [P.info.obstacles, {world_info.obstacles}] ;
    P.info.q_0 = [P.info.q_0, {q_0}] ;
    P.info.q_dot_0 = [P.info.q_dot_0, {q_dot_0}] ;
    P.info.k_opt = [P.info.k_opt, {k_opt}] ;
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
        P.info.FO_zono = [P.info.FO_zono, {FO_zono}];
        P.info.sliced_FO_zono = [P.info.sliced_FO_zono, {sliced_FO_zono}];
    end

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