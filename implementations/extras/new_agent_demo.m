
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

%% Initialize the visual and collision systems
visual = PatchVisualSystem(dynamic_objects=A_3.visual);
collision = Patch3dCollisionSystem(dynamic_objects=A_3.collision);

% Create the base obstacles
base_creation_buffer = 0.025;
face_color = [0.5 0.5 0.5];
edge_color = [0 0 0];

base_options.info.is_base_obstacle = true;
base_options.info.creation_buffer = base_creation_buffer;
base_options.visual.face_color = face_color;
base_options.visual.edge_color = edge_color;
optionsStruct.component_options = base_options;
base = BoxObstacle.makeBox( [-0.0580; 0; 0.1778], ...
                            2*[0.2794, 0.2794, 0.1778], ...
                            optionsStruct);
tower = BoxObstacle.makeBox([-0.2359; 0; 0.6868], ...
                            2*[0.1016, 0.1651, 0.3312], ...
                            optionsStruct);
head = BoxObstacle.makeBox( [-0.0580; 0; 1.0816], ...
                            2*[0.1651, 0.1397, 0.0635], ...
                            optionsStruct);
% Floor
floor_color = [0.9, 0.9, 0.9];
optionsStruct.component_options.visual.face_color = face_color;
floor = BoxObstacle.makeBox([-0.0331;0;0.005], ...
                            2*[1.3598, 1.3598, 0.0025], ...
                            optionsStruct);

% Add them to the collision
collision.add_staticObjects(base.collision.get_patch3dObject, ...
                            tower.collision.get_patch3dObject, ...
                            head.collision.get_patch3dObject, ...
                            floor.collision.get_patch3dObject)

visual.addObjects(static_objects = [base.visual, ...
                  tower.visual, ...
                  head.visual, ...
                  floor.visual])

% Create a random start (assuming no obstacles)
randomizing = true;
while randomizing
    A_3.state.random_init();
    proposal_obj = A_3.collision.get_patch3dObject();

    % test it in the collision system
    [randomizing, pairs] = collision.checkCollisionObject(proposal_obj);
end
% This is captured by the goal generator if we don't set anything as the
% start.

% Create the random obstacles
n_obstacles = 10;
obstacle_size_range = [0.01 0.5] ; % [min, max] side length
creation_buffer = 0.05;
world_bounds = [A_3.info.reach_limits(1:2:6); A_3.info.reach_limits(2:2:6)];
obstacles = [];
for obs_num = 1:n_obstacles
    randomizing = true;
    while randomizing
        % create center, side lengths
        center = rand_range( world_bounds(1,:) + obstacle_size_range(2)/2,...
                             world_bounds(2,:) - obstacle_size_range(2)/2 );
        side_lengths = rand_range(obstacle_size_range(1),...
                                  obstacle_size_range(2),...
                                  [],[],...
                                  1, 3); % 3 is the dim of the world in this case
        % Create obstacle
        optionsStruct = struct;
        optionsStruct.component_options.info.creation_buffer = base_creation_buffer;
        prop_obs = BoxObstacle.makeBox(center, side_lengths, optionsStruct);

        % test it
        proposal_obj = prop_obs.collision.get_patch3dObject();

        % test it in the collision system
        [randomizing, pairs] = collision.checkCollisionObject(proposal_obj);
    end
    % if it's good, we save the proposal_obj
    collision.add_staticObjects(proposal_obj);
    visual.addObjects(static_objects=prop_obs.visual);
    obstacles = [obstacles; prop_obs];
end

% Create and add the goal
goal = RandomArmConfigurationGoal(collision, A_3);
goal.reset();
goal.createGoal();
visual.addObjects(static_objects=goal);

% redraw
visual.redraw();