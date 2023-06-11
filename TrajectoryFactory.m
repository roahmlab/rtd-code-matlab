classdef TrajectoryFactory < Trajectory
    methods(Static)

        function trajectories = CreateTrajectories(au_values,ay_values,u0_goal_val,manu_type_val,u0_val,t0_offset_val,h0_val)
            num_trajectories = numel(au_values);
            trajectories = cell(1,num_trajectories);
            for i = 1:num_trajectories
                au = au_values(i);
                ay = ay_values(i);
                u0_goal = u0_goal_val(i);
                manu_type_i = manu_type_val(i);
                u0 = u0_val(i);
                t0_offset = t0_offset_val(i);
                h0 = h0_val(i);
                trajectories{i} = Trajectory(au,ay,u0_goal,manu_type_i,u0,t0_offset,h0);
            end

        end
    end


end