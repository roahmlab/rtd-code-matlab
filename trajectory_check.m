% clear all;
% clc

au = 2.0;
ay = 1.5;
u0_goal = 5.0;
manu_type = 'type1';
u0 = 1.0;
t0_offset = 0.2;
h0 = 0.1;

traj = Trajectory.CreateTrajectory(au, ay, u0_goal, manu_type, u0, t0_offset, h0);
