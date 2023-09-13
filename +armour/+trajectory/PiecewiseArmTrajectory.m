classdef PiecewiseArmTrajectory < rtd.trajectory.Trajectory
% Piecewise parameterized trajectory for the arm robot
%
% This trajectory is a piecewise trajectory with a plan time and a horizon
% time. The trajectory accelerates for the plan time, and then decelerates
% until the horizon time. The trajectory is parameterized by the acceleration
% during the plan time, and the trajectory is computed from the initial
% state of the robot.
%
% The trajectory is parameterized by the following:
%   startState: the initial state of the robot
%   planTime: the time to accelerate for
%   horizonTime: the time after starting that the robot should be stopped by
%   params: the parameters of the trajectory, which are the accelerations
%       during the plan time. This can also be scaled by a RangeScaler
%       object.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2022-09-08
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
        % The initial state of the trajectory. These are initial parameters
        % from the robot used to compute the trajectory.
        startState(1,1) rtd.entity.states.ArmRobotStateInstance

        % The plan time is the duration the trajectory accelerates at the
        % parameterized value for before executing the braking action.
        planTime(1,1) double

        % The horizon time is the time at which the trajectory will stop
        % after starting, after which the trajectory will hold the final
        % position.
        horizonTime(1,1) double

        % Range scaling object for the trajectory parameters
        paramScale rtd.util.RangeScaler = rtd.util.RangeScaler.empty()

        % the number of parameters to use for the trajectory
        numParams(1,1) uint32
    end

    % Internal properties (for precomputed values)
    properties (SetAccess=private)
        % the acceleration for the plan time portion of the trajectory
        q_ddot(:,1) double
        
        % the peak position of the trajectory
        q_peak(:,1) double
        
        % the peak velocity of the trajectory
        q_dot_peak(:,1) double
        
        % the acceleration for the stopping portion of the trajectory (horizon time - plan time)
        q_ddot_to_stop(:,1) double

        % the end position of the trajectory
        q_end(:,1) double
    end

    methods
        function self = PiecewiseArmTrajectory(startState, planTime, horizonTime, numParams)
            % Constructor for the PiecewiseArmTrajectory class
            %
            % Arguments:
            %   startState (rtd.entity.states.ArmRobotStateInstance): the initial state of the robot
            %   planTime (double): the time to accelerate for
            %   horizonTime (double): the time after starting that the robot should be stopped by
            %   numParams (uint32): the number of parameters to use for the trajectory
            %
            arguments
                startState(1,1) rtd.entity.states.ArmRobotStateInstance
                planTime(1,1) double
                horizonTime(1,1) double
                numParams(1,1) uint32
            end

            self.startState = startState;
            self.planTime = planTime;
            self.horizonTime = horizonTime;
            self.numParams = numParams;
        end

        function setParamScale(self, paramScale)
            % Sets the parameter scaling
            %
            % Arguments:
            %   paramScale (rtd.util.RangeScaler): The parameter scaling to use.
            %       This is used to scale the relative position to the desired
            %       position. If not provided, then paramScale is reset to empty.
            %
            arguments
                self armour.trajectory.PiecewiseArmTrajectory
                paramScale(:,1) rtd.util.RangeScaler = []
            end

            if length(paramScale) > 1
                error('Can only set one paramScale value!');
            end
            self.paramScale = paramScale;
        end
        
        function setParameters(self, trajectoryParams, options)
            % Sets the parameters of the trajectory
            %
            % After setting the parameters, the internal parameters are
            % updated.
            %
            % Arguments:
            %   trajectoryParams (double): the parameters of the trajectory, which are the
            %       accelerations during the plan time. This can also be scaled by
            %       a RangeScaler object.
            %   options: Keyword Arguments. See below.
            %
            % Keyword Arguments:
            %   startState (rtd.entity.states.ArmRobotStateInstance): the initial state of the robot
            %   planTime (double): the time to accelerate for
            %   horizonTime (double): the time after starting that the robot should be stopped by
            %   numParams (uint32): the number of parameters to use for the trajectory
            %   paramScale (rtd.util.RangeScaler): The parameter scaling to use.
            %       This is used to scale the relative position to the desired position.
            %
            arguments
                self armour.trajectory.PiecewiseArmTrajectory
                trajectoryParams(1,:) double
                options.startState rtd.entity.states.ArmRobotStateInstance = self.startState
                options.planTime(1,1) double = self.planTime
                options.horizonTime(1,1) double = self.horizonTime
                options.numParams(1,1) uint32 = self.numParams
                options.paramScale(:,1) rtd.util.RangeScaler = self.paramScale
            end

            self.trajectoryParams = trajectoryParams;
            self.startState = options.startState;
            self.planTime = options.planTime;
            self.horizonTime = options.horizonTime;
            self.numParams = options.numParams;
            if length(self.trajectoryParams) > self.numParams
                self.trajectoryParams = self.trajectoryParams(1:self.numParams);
            end
            self.setParamScale(options.paramScale);
            
            % Perform an internal update (compute peak and stopping values)
            self.internalUpdate();
        end
        
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
            %   valid (logical): True if the trajectory parameters are valid.
            %       False otherwise.
            %
            % Raises:
            %   InvalidTrajectory: If the current trajectory is not valid and
            %       throwOnError is true.
            %
            arguments
                self armour.trajectory.PiecewiseArmTrajectory
                throwOnError(1,1) logical = false
            end

            % Make sure everything is nonempty
            valid = not( ...
                isempty(self.trajectoryParams) || ...
                isempty(self.startState));

            % Make sure the trajectory params make sense
            valid = valid && length(self.trajectoryParams) == self.numParams;
            valid = valid && self.numParams == length(self.startState.position);
            
            % Throw if wanted
            if ~valid && throwOnError
                errMsg = MException('PiecewiseArmTrajectory:InvalidTrajectory', ...
                    'Called trajectory object does not have complete parameterization!');
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
                self armour.trajectory.PiecewiseArmTrajectory
                time(1,:) double
            end

            % Do a parameter check and time check, and throw if anything is
            % invalid.
            self.validate(true);
            t_shifted = time - self.startTime;
            if t_shifted < 0
                ME = MException('PiecewiseArmTrajectory:InvalidTrajectory', ...
                    'Invalid time provided to PiecewiseArmTrajectory');
                throw(ME)
            end

            % Mask the first and second half of the trajectory
            t_plan_mask = t_shifted < self.planTime;
            t_stop_mask = logical((t_shifted < self.horizonTime) - t_plan_mask);
            t_plan_vals = t_shifted(t_plan_mask);
            t_stop_vals = t_shifted(t_stop_mask) - self.planTime;

            % Create the combined state variable
            t_size = length(time);
            pos_idx = 1:self.numParams;
            vel_idx = pos_idx + self.numParams;
            acc_idx = vel_idx + self.numParams;
            state = zeros(self.numParams*3, t_size);
            
            % Rename variables
            q_0 = self.startState.position;
            q_dot_0 = self.startState.velocity;

            % Compute the first half of the trajectory
            if any(t_plan_mask)
                state(pos_idx,t_plan_mask) = q_0 + ...
                                q_dot_0 * t_plan_vals + ...
                                (1/2) * self.q_ddot * t_plan_vals.^2;
                state(vel_idx,t_plan_mask) = q_dot_0 + ...
                                self.q_ddot * t_plan_vals;
                state(acc_idx,t_plan_mask) = repmat(self.q_ddot, 1, length(t_plan_vals));
            end
            
            % Compute the second half of the trajectory
            if any(t_stop_mask)
                state(pos_idx,t_stop_mask) = self.q_peak + ...
                                self.q_dot_peak * t_stop_vals + ...
                                (1/2) * self.q_ddot_to_stop * t_stop_vals.^2;
                state(vel_idx,t_stop_mask) = self.q_dot_peak + ...
                                self.q_ddot_to_stop * t_stop_vals;
                state(acc_idx,t_stop_mask) = repmat(self.q_ddot_to_stop, 1, length(t_stop_vals));
            end

            % Update all states after the horizon time
            state(pos_idx,~(t_plan_mask+t_stop_mask)) = repmat(self.q_end, 1, t_size-sum(t_plan_mask+t_stop_mask));

            % Generate the output.
            command(length(time)) = rtd.entity.states.ArmRobotStateInstance();
            command.setTimes(time);
            command.setStateSpace(state, ...
                position_idxs=pos_idx, ...
                velocity_idxs=vel_idx, ...
                acceleration_idxs=acc_idx);
        end
    end

    % Internal methods for the class
    methods (Access=private)
        function internalUpdate(self)
            % Performs an internal update of the trajectory.
            %
            % This function performs an internal update of the trajectory. This
            % includes calculating the bernstein coefficients and the final position.
            % This function is called automatically when the trajectory parameters
            % are set.
            %

            % internal update if valid. short circuit if not.
            if ~self.validate()
                return
            end
            
            % Rename variables
            q_0 = self.startState.position;
            q_dot_0 = self.startState.velocity;
            
            % Scale the parameters if needed
            self.q_ddot = self.trajectoryParams;
            if isempty(self.paramScale)
                self.q_ddot = self.trajectoryParams;
            else
                self.q_ddot = self.paramScale.scaleout(self.trajectoryParams);
            end
            
            % Compute the peak parameters
            self.q_peak = q_0 + ...
                q_dot_0 * self.planTime + ...
                (1/2) * self.q_ddot * self.planTime^2;
            self.q_dot_peak = q_dot_0 + ...
                self.q_ddot * self.planTime;

            % Compute the stopping parameters
            stopping_time = (self.horizonTime - self.planTime);
            self.q_ddot_to_stop = (0-self.q_dot_peak) / stopping_time;
            self.q_end = self.q_peak + ...
                self.q_dot_peak * stopping_time + ...
                (1/2) * self.q_ddot_to_stop * stopping_time^2;

            self.startTime = self.startState.time;
        end
    end
end