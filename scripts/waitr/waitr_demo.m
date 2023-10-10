
clear ; clc; close all;

%% Agent parameters
verbosity = 'DEBUG';

%%% for agent
agent_urdf = 'Kinova_Grasp_URDF.urdf';

add_uncertainty_to = 'link'; % choose 'all', 'link', or 'none'
links_with_uncertainty = {'cube_link'}; % if add_uncertainty_to = 'link', specify links here.
uncertain_mass_range = [0.97, 1.03];
u_s = 0.609382421; 
surf_rad = 0.029;

% If this flag is true, we use the ArmourCadPatchVisual component instead
visualize_cad = false;

% Add noise to the dynamics
component_options.dynamics.measurement_noise_points = 0;
component_options.dynamics.log_controller = true;
component_options.dynamics.u_s = u_s;
component_options.dynamics.surf_rad = surf_rad;

% for controller
component_options.controller.use_true_params_for_robust = false;

%% Setup the info
robot = importrobot(agent_urdf);
robot.DataFormat = 'col';
robot.Gravity = [0 0 -9.81];
params = armour.legacy.dynamics.load_robot_params(robot, ...
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

%controller = [];
controller = 'waitr.agent.WaitrMexController';

% Create
agent = waitr.WaitrAgent(agent_info, visual=visual, ...
                component_options=component_options, ...
                component_logLevelOverride=verbosity, ...
                controller=controller);

close all;

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
sim = waitr.WaitrSimulation;
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
trajOptProps.timeForCost = 0.5;

input_constraints_flag = true;
grasp_constraints_flag = true;
use_robust_input = true;
smooth_obs = false;
joint_limits_flag = true;

planner = waitr.WaitrPlanner( ...
        trajOptProps, sim.agent, ...
        input_constraints_flag=input_constraints_flag,...
        grasp_constraints_flag=grasp_constraints_flag,...
        use_robust_input=use_robust_input,...
        smooth_obs=smooth_obs, ...
        joint_limits_flag=joint_limits_flag, ...
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

% Using callbacks to attach to the sim
cb = @(sim) planner_callback(sim, planner, agent_info, world_info, lookahead, HLP);
sim.run(max_steps=100, pre_step_callback={cb}, autolog=true);

% % How to use the listeners to attach to the sim instead of callbacks
% cb = @(sim, event) planner_callback(sim, planner, agent_info, world_info, lookahead, HLP);
% addlistener(sim, 'Step', cb);
% sim.run(max_steps=100, autolog=true);

%% Evaluate simulation
% sim.visual_system.animate();
enable_compare = false;
if enable_compare
    load("saved_data_case1.mat");
end

% Plot joint positions, velocities, and torques
entries = agent.dynamics.controller_log.get('input_time', 'input', flatten=false);
inputs = [];
for i = 1:size(entries.input,2)
    temp_input = entries.input{i};
    if i ~= size(entries.input,2)
        temp_input = temp_input(:,1:(end-1));
    end
    inputs = [inputs, temp_input];
end

figure(2);
for joint_id = 1:7
    subplot(2,4,joint_id);
    plot(agent.state.time,agent.state.position(joint_id,:),'b'); hold on;
    if enable_compare
        plot(agent.state.time,saved_data.pos(joint_id,:),'g');
    end
    xlabel('time (s)');
    ylabel('state (rad)');
    title(['Joint #',num2str(joint_id)]);
    if enable_compare && joint_id == 1
        legend("Use code platform","ARMOUR's original code");
    end
end
sgtitle('Joint Positions');

figure(3);
for joint_id = 1:7
    subplot(2,4,joint_id);
    plot(agent.state.time,agent.state.velocity(joint_id,:), 'b'); hold on;
    if enable_compare
        plot(agent.state.time,saved_data.vel(joint_id,:),'g');
    end
    xlabel('time (s)');
    ylabel('velocity (rad/s)');
    title(['Joint #',num2str(joint_id)]);
    if enable_compare && joint_id == 1
        legend("Use code platform","ARMOUR's original code");
    end
end
sgtitle('Joint Velocities');

figure(4);
for joint_id = 1:7
    subplot(2,4,joint_id);
    plot(agent.state.time,inputs(joint_id,:), 'b'); hold on;
    if enable_compare
        plot(agent.state.time,saved_data.torque(joint_id,:),'g');
    end
    xlabel('time (s)');
    ylabel('torque (Nm)');
    title(['Joint #',num2str(joint_id)]);
    if enable_compare && joint_id == 1
        legend("Use code platform","ARMOUR's original code");
    end
end
sgtitle('Joint Torques');

% Calculating the Acceleration
qdd_post = zeros(7,length(agent.state.time));
% calculating the acceleration in post to compare with what is stored
for i = 1:length(agent.state.time)
    t = agent.state.time(i);
    z = agent.state.state(:,i);
    [M, C, g] = agent.dynamics.calculate_dynamics(agent.state.position(:,i), agent.state.velocity(:,i), agent.dynamics.robot_info.params.true);
    for j = 1:agent_info.n_inputs
        M(j,j) = M(j,j) + agent.dynamics.robot_info.transmission_inertia(j);
    end
    qdd_post(:,i) = M\(inputs(:,i)-C*agent.state.velocity(:,i)-g);
end

figure(5);
for joint_id = 1:7
    subplot(2,4,joint_id);
    plot(agent.state.time,qdd_post(joint_id,:), 'b'); hold on;
    if enable_compare
        plot(agent.state.time,saved_data.acc(joint_id,:),'g');
    end
    xlabel('time (s)');
    ylabel('acceleration (rad/s^2)');
    title(['Joint #',num2str(joint_id)]);
    if enable_compare && joint_id == 1
        legend("Use code platform","ARMOUR's original code");
    end
end
sgtitle('Joint Accelerations');

% Calling RNEA
for i = 1:length(agent.state.time)
    % clear relevant variables
    clear tau f n
    % call rnea
    [tau, f, n] = armour.legacy.dynamics.rnea(agent.state.position(:,i)',agent.state.velocity(:,i)',agent.state.velocity(:,i)',qdd_post(:,i)',true,agent.dynamics.robot_info.params.true);
    % store rnea results
    Tau{i} = tau;
    F{i} = f;
    N{i} = n;
    % store the contact forces
    force(:,i) = f(:,10);
    % store the contact moments
    moment(:,i) = n(:,10);
end

% Plotting the forces
figure(6);
% plot the x-axis force (in the tray frame)
subplot(1,3,1);
hold on;
plot(agent.state.time(1:end), force(1,:), 'b');
if enable_compare
    plot(agent.state.time(1:end), saved_data.force(1,:), 'g');
end
xlabel('time (s)');
ylabel('x-axis Force (N)');
axis('square');
grid on;
if enable_compare
    legend("Use code platform","ARMOUR's original code");
end
% plot the y-axis force (in the tray frame)
subplot(1,3,2);
hold on;
plot(agent.state.time(1:end), force(2,:), 'b');
if enable_compare
    plot(agent.state.time(1:end), saved_data.force(2,:), 'g');
end
xlabel('time (s)');
ylabel('y-axis Force (N)');
axis('square');
grid on;
% plot the z-axis force (in the tray frame)
subplot(1,3,3);
hold on;
plot(agent.state.time(1:end), force(3,:), 'b');
if enable_compare
    plot(agent.state.time(1:end), saved_data.force(3,:), 'g');
end
xlabel('time (s)');
ylabel('z-axis Force (N)');
axis('square');
grid on;
sgtitle('Contact Force');

% Calculating Constraints
% separation constraint
sep = -1*force(3,:);
% slipping constraint
% slip = sqrt(force(1,:).^2+force(2,:).^2) - u_s.*abs(force(3,:));
slip2 = force(1,:).^2+force(2,:).^2 - u_s^2.*force(3,:).^2;

for i = 1:length(agent.state.time)
    % tipping constraint the normal way
    ZMP_top = cross([0;0;1],moment(:,i)); % normal vector should come first
    ZMP_bottom = dot([0;0;1],force(:,i));
    ZMP(:,i) = ZMP_top/ZMP_bottom;
    ZMP_rad(i) = sqrt(ZMP(1,i)^2+ZMP(2,i)^2);
%     tip(i) = ZMP_rad(i) - surf_rad;
    % tipping constraint the PZ way
    ZMP_top2 = cross([0;0;1],moment(:,i));
    ZMP_bottom2 = dot([0;0;1],force(:,i));
    tip2(i) = ZMP_top(1)^2 + ZMP_top(2)^2 - ZMP_bottom^2*(surf_rad)^2;
end

% plotting grasp constraints
figure(7);
% plot the separation constraint
subplot(1,3,1);
hold on;
plot(agent.state.time(1:end),sep, 'b');
if enable_compare
    plot(agent.state.time(1:end), saved_data.sep, 'g');
end
plot(agent.state.time(1:end), zeros(1,length(agent.state.time)), 'r');
xlabel('time (s)');
ylabel('Separation Constraint');
axis('square');
grid on;
if enable_compare
    legend("Use code platform","ARMOUR's original code","Constraint limit");
else
    legend("Use code platform","Constraint limit");
end
% plot the slipping constraint
subplot(1,3,2);
hold on
%     plot(agent.state.time(1:end),slip, 'k')
plot(agent.state.time(1:end),slip2, 'b');
if enable_compare
    plot(agent.state.time(1:end), saved_data.slip2, 'g');
end
plot(agent.state.time(1:end), zeros(1,length(agent.state.time)), 'r');
xlabel('time (s)');
ylabel('Slipping Constraint');
axis('square');
grid on;
% plot the tipping constraint
subplot(1,3,3);
hold on;
% plot(agent.state.time(1:end),tip, 'k')
plot(agent.state.time(1:end),tip2, 'b');
if enable_compare
    plot(agent.state.time(1:end), saved_data.tip2, 'g');
end
plot(agent.state.time(1:end), zeros(1,length(agent.state.time)), 'r');
xlabel('time (s)');
ylabel('Tipping Constraint');
axis('square');
grid on;
sgtitle('Grasp Constraints');

%% Save simulation data
saved_data = struct();
saved_data.pos = agent.state.position;
saved_data.vel = agent.state.velocity;
saved_data.torque = inputs;
saved_data.acc = qdd_post;
saved_data.force = force;
saved_data.sep = sep;
saved_data.slip2 = slip2;
saved_data.tip2 = tip2;
saved_data.goal_pos = sim.goal_system.goal_position;
saved_data.obstacles = sim.obstacles;
savename = sprintf('comparison_result/saved_data_arch_%s.mat', datestr(now,'mm_dd_yyyy_HH_MM_SS'));
save(savename,"saved_data");



%% Helper functions
function info = planner_callback(sim, planner, agent_info, world_info, lookahead, HLP)
    % Get the end state
    time = sim.agent.state.time(end);
    ref_state = sim.agent.controller.trajectories.getCommand(time);
    agent_info.state = sim.agent.state.state(:,end);

    q_des = HLP.get_waypoint(agent_info,world_info,lookahead);
    if isempty(q_des)
        disp('Waypoint creation failed! Using global goal instead.')
        q_des = HLP.goal ;
    end

    % get the sensor readings at the time
    worldState.obstacles = rtd.sim.sensors.zonotope_sensor(sim.world.all_entities, sim.agent, time);
    [trajectory, plan_info] = planner.planTrajectory(ref_state, worldState, q_des);
    %FO = plan_info.rsInstances{2}.FO;
    %jrsinfo = plan_info.rsInstances{1}.jrs_info;
    
    if ~isempty(trajectory)
        sim.agent.controller.setTrajectory(trajectory)
    end

    info = plan_info;
end