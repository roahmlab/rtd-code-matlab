classdef ZeroHoldTrajectory < Trajectory
    % ArmTdTrajectory
    % The original ArmTD trajectory with peicewise accelerations
    properties (Constant)
        % Size of the trajectoryParams, we want this set for all use
        param_shape = 7; % This already needs a change!
        % TODO - update to add dynamic parameter as an option
    end
    properties
        % The starting state of the robot
        robotState
    end
    methods
        % The ArmTdTrajectory constructor, which simply sets parameters and
        % attempts to call internalUpdate, a helper function made for this
        % class to update all other internal parameters once fully
        % parameterized.
        function self = ZeroHoldTrajectory(    ...
                    robotState,             ...
                    varargin                ...
                )
            self.robotState = robotState;
        end
        
        % Set the parameters of the trajectory, with a focus on the
        % parameters as the state should be set from the constructor.
        function setTrajectory(         ...
                    self,               ...
                    robotState,         ...
                    varargin            ...
                )
            self.robotState = robotState;
        end
        
        % Validate that the trajectory is fully characterized
        function valid = validate(self, throwOnError)
            valid = true;
        end
        
        % Computes the actual input commands for the given time.
        % throws RTD:InvalidTrajectory if the trajectory isn't set
        % throws RTD:Trajectory:InvalidTime if the time is before the
        % trajectory exists
        % TODO: write in a vectorized manner
        function command = getCommand(self, time)
            % Validate, if invalid, throw
            self.validate(true);
            % TODO: throw invalid trajectory
            t = time - self.robotState.time;
            
            % Ensure time is valid
            if t < 0
                ME = MException('RTD:Trajectory:InvalidTime', ...
                    'Invalid time provided to ArmTdTrajectory');
                throw(ME)
            end
            
            q_des = self.robotState.q;
            q_dot_des = self.robotState.q*0;
            q_ddot_des = self.robotState.q*0;

            n_q = length(q_des);
            command = ArmRobotTrajectoryState(1:n_q, n_q+1:n_q*2, n_q*2+1:n_q*3);
            command.time = time;
            command.state = [q_des; q_dot_des; q_ddot_des];
        end
    end
end