classdef Trajectory_Factory < rtd.planner.trajectory.TrajectoryFactory
    properties
        AH
    end
    methods

        function self = Trajectory_Factory(AH)
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

            disp(class(parameter))
            disp(parameter)
            au = parameter(1);
            ay = parameter(2);
            u0_goal = 4; %dummy
            manu_type = 'speed change';
            u0 = 4;
            t0_offset =0;
            h0=0;

            if nargin < 5
                u0 = 0.0;
            end
            if nargin < 6
                t0_offset = 0.0;
            end
            if nargin < 7
                h0 = 0.0;
            end
            
            % Create an instance of the Trajectory class
            K = [2,3,2,1];
            agent_state = [4,5,3,5];
            traj = Trajectory_reference(self.AH, K, agent_state);% AgentHelper, K=[Au,Ay,t0_idx,Manu_type], agent_state = [x_curr,y_curr,h_curr,u_curr]
            
            % Set the trajectory parameters
            traj.trajectoryParams = 0; % Specify the trajectory parameters here
            
            % Set the trajectory optimization properties
            % DUmmy variables for testing
            trajOptProps = rtd.planner.trajopt.TrajOptProps();
            trajOptProps.timeForCost = 1.0; % Set the time used for evaluation in the cost function
            trajOptProps.planTime = 0.5; % Set the time duration of the nominal plan
            trajOptProps.horizonTime = 1.0; % Set the time of the overall trajectory until stop
            trajOptProps.doTimeout = false; % Set whether or not to timeout the optimization
            trajOptProps.timeoutTime = 0.5; % Set the timeout time for the optimization
            trajOptProps.randomInit = false; % Set whether or not to randomize unknown or extra parameters
            traj.trajOptProps = trajOptProps; % Specify the trajectory optimization properties here
            
            % Set the start state
            traj.startState = rtd.entity.states.EntityState(); % Specify the start state here
            
            % Call the setParameters method
            traj.setParameters(au, ay, u0_goal, manu_type, u0, t0_offset, h0);
        end
    end


end