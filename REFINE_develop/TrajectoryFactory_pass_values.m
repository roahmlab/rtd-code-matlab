classdef TrajectoryFactory_pass_values  < rtd.planner.trajectory.TrajectoryFactory

    %trajectory factory pass through function

    properties
        AH
    end
    methods
        function self = TrajectoryFactory_pass_values(AH)
            self.AH = AH;
        end

        function traj = createTrajectory(self,robotState,rsInstance,parameter)

            %the above vehrs and robotState is only of the desired idx
            %au_values: longitudnal velocity ->parameter
            %ay_values: lateral velocity
            %u0_goal: goal longitudnal velocity
            %manu_type_val: maneuver type
            %t0_offset: time offset for a trajectory
            %u0_val:initial longitudnal velocity of the robot
            %h0_val: initial head of the robot


            manu_type = rsInstance.frs_.manu_type;
          
            
            % Create an instance of the Trajectory class
            K = [2,3,2,0];

            if strcmp(manu_type, 'speed_change')
                K(4) = 1;
            elseif strcmp(manu_type,'lane_change')
                K(4) = 3;
            end

            agent_state = [4,5,3,5];
            [T ,U ,Z ] = gen_ref(self.AH, K,agent_state);
            traj = Z;
        end
    end


end