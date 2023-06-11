au_values = [2.0, 1.5, 3.0];
ay_values = [1.5, 2.0, 1.0];
u0_goal_values = [5.0, 3.0, 4.0];
manu_type_values = {'type1', 'type2', 'type1'};
u0_values = [1.0, 0.5, 0.0];
t0_offset_values = [0.2, 0.0, 0.1];
h0_values = [0.1, 0.2, 0.0];

trajectories = TrajectoryFactory.CreateTrajectories(au_values, ay_values, u0_goal_values, manu_type_values, u0_values, t0_offset_values, h0_values);
