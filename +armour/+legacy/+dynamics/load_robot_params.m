function [params] = load_robot_params(robot,varargin)
% given a robot and a set of options, get all sets of params (nominal,
% true, interval, PZ).

% adam 20230206
% copied into legacy subpackage of armour in rtd-code repository from
% commit df7cc3e and adapted
%

p.add_uncertainty_to = 'all';
p.links_with_uncertainty = {};
p.true_mass_range = [1.01, 1.01]; % make sure first and second column are equal... note that if add_uncertainty_to = 'link', only changes the params of that link!
p.true_com_range = [1, 1];
p.uncertain_mass_range = [0.97, 1.03];
p.uncertain_com_range  = [1, 1]; % default
p.track_inertial_generators = false;
p.zono_order = 40;

p = parse_args(p, varargin{:});

% get robot parameters:
kinematic_params = get_kinematic_params(robot);

nominal_inertial_params = get_inertial_params(robot, 'set_type', 'point',...
                                                          'add_uncertainty_to', 'none');

true_inertial_params = get_inertial_params(robot, 'set_type', 'point',...
                                                          'add_uncertainty_to', p.add_uncertainty_to,...
                                                          'links_with_uncertainty', p.links_with_uncertainty,...
                                                          'mass_range', p.true_mass_range,...
                                                          'com_range', p.true_com_range);

interval_inertial_params = get_inertial_params(robot, 'set_type', 'interval',...
                                                          'add_uncertainty_to', p.add_uncertainty_to,...
                                                          'links_with_uncertainty', p.links_with_uncertainty,...
                                                          'mass_range', p.uncertain_mass_range,...
                                                          'com_range', p.uncertain_com_range);

pz_nominal_inertial_params = get_inertial_params(robot, 'set_type', 'polynomial_zonotope',...
                                                          'add_uncertainty_to', 'none',...
                                                          'zono_order', p.zono_order);

pz_interval_inertial_params = get_inertial_params(robot, 'set_type', 'polynomial_zonotope',...
                                                          'add_uncertainty_to', p.add_uncertainty_to,...
                                                          'links_with_uncertainty', p.links_with_uncertainty,...
                                                          'mass_range', p.uncertain_mass_range,...
                                                          'com_range', p.uncertain_com_range,...
                                                          'track_inertial_generators', p.track_inertial_generators,...
                                                          'zono_order', p.zono_order);

% merge kinematic and inertial params:
params.nominal = merge_param_structs(kinematic_params, nominal_inertial_params);
params.true = merge_param_structs(kinematic_params, true_inertial_params);
params.interval = merge_param_structs(kinematic_params, interval_inertial_params);
params.pz_interval = merge_param_structs(kinematic_params, pz_interval_inertial_params);
params.pz_nominal = merge_param_structs(kinematic_params, pz_nominal_inertial_params);

end

