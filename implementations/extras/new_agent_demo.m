
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