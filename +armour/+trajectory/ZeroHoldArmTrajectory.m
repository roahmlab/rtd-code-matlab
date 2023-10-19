classdef ZeroHoldArmTrajectory < rtd.trajectory.Trajectory
% An arm trajectory that holds the arm at a fixed position and zero velocity/acceleration.
%
% This acts as the initial trajectory for the arm, and is used to hold the
% arm in place while the robot is waiting for the user to start the
% trajectory.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-01-05
% Updated: 2023-09-12 (Adam Li)
%
% See also: rtd.trajectory.Trajectory, armour.trajectory.ArmTrajectoryFactory,
% rtd.entity.states.ArmRobotStateInstance
%
% --- More Info ---
%

    % Required properties
    properties (Constant)
        % True as this trajectory supports vectorized computation
        vectorized = true
    end

    % Additional properties
    properties
        % The initial state of the trajectory. This is the state that the
        % trajectory will hold the arm position at.
        startState(1,1) rtd.entity.states.ArmRobotStateInstance
    end

    methods
        function self = ZeroHoldArmTrajectory(startState)
            % The constructor for the ZeroHoldArmTrajectory class.
            %
            % Arguments:
            %   startState: The initial state of the trajectory. This is the state that the
            %       trajectory will hold the arm position at.
            %
            arguments
                startState(1,1) rtd.entity.states.ArmRobotStateInstance
            end

            self.startState = startState;
            self.startTime = self.startState.time;
        end
        
        % Set the parameters of the trajectory, with a focus on the
        % parameters as the state should be set from the constructor.
        function setParameters(self, trajectoryParams, options)
            % Set the parameters of the trajectory.
            %
            % Arguments:
            %   trajectoryParams: The parameters of the trajectory, which is ignored
            %       for this trajectory.
            %   options: Keyword arguments. See below.
            %
            % Keyword Arguments:
            %   startState: The initial state of the trajectory. This is the state that the
            %       trajectory will hold the arm position at.
            %
            arguments
                self armour.trajectory.ZeroHoldArmTrajectory
                trajectoryParams(1,:) double
                options.startState rtd.entity.states.ArmRobotStateInstance = self.startState
            end

            self.trajectoryParams = trajectoryParams;
            self.startState = options.startState;
            self.startTime = self.startState.time;
        end
        
        % Validate that the trajectory is fully characterized
        function valid = validate(self, throwOnError)
            % Validates the trajectory parameters.
            %
            % This function ensures that the trajectory parameters are valid and fully
            % specified. If the trajectory parameters are invalid, then it either throws
            % an error or returns false.
            %
            % Arguments:
            %   throwOnError (logical, optional): If true, then throws an error if the
            %       trajectory parameters are invalid. Otherwise, returns false.
            %       Defaults to false.
            %
            % Returns:
            %   valid (logical): Whether or not the trajectory parameters are valid.
            %
            % Raises:
            %   InvalidTrajectory: If the current trajectory is not valid and
            %       throwOnError is true.
            %
            arguments
                self armour.trajectory.ZeroHoldArmTrajectory
                throwOnError(1,1) logical = false
            end

            % Make sure we actually have a robot state to work with.
            valid = ~isempty(self.startState.position);

            % Throw if wanted
            if ~valid && throwOnError
                errMsg = MException('ZeroHoldArmTrajectory:InvalidTrajectory', ...
                    'Must have some existing robot state to use this!');
                throw(errMsg)
            end
        end
        
        function command = getCommand(self, time)
            % Gets the command for the given time.
            %
            % This function gets the command for the given time. The command is
            % generated by calculating the position, velocity, and acceleration
            % for the given time.
            %
            % Arguments:
            %   time (double): The time to get the command for.
            %
            % Returns:
            %   command (rtd.entity.states.ArmRobotStateInstance): The command for the given time.
            %
            % Raises:
            %   InvalidTrajectory: If the current trajectory is not valid.
            %   InvalidTime: If the time is invalid.
            %
            arguments
                self armour.trajectory.ZeroHoldArmTrajectory
                time(1,:) double
            end

            % Do a parameter check and time check, and throw if anything is
            % invalid.
            self.validate(true);
            if any(time < self.startTime)
                ME = MException('ZeroHoldArmTrajectory:InvalidTime', ...
                    'Invalid time provided to ZeroHoldArmTrajectory');
                throw(ME)
            end
            
            % Make the state
            n_q = length(self.startState.position);
            state = repmat([self.startState.position;0], 1, length(time));
            pos_idx = 1:n_q;
            acc_vel_idx = ones(1,n_q)+n_q;

            % Generate the output.
            command(length(time)) = rtd.entity.states.ArmRobotStateInstance();
            command.setTimes(time);
            command.setStateSpace(state, ...
                position_idxs=pos_idx, ...
                velocity_idxs=acc_vel_idx, ...
                acceleration_idxs=acc_vel_idx);
        end
    end
end