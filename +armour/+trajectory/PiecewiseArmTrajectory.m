classdef PiecewiseArmTrajectory < rtd.trajectory.Trajectory
    % PiecewiseArmTrajectory
    % The original ArmTD trajectory with peicewise accelerations
    % Required properties
    properties (Constant)
        vectorized = true
    end
    % Additional properties
    properties
        startState(1,1) rtd.entity.states.ArmRobotStateInstance
        planTime
        horizonTime

        % Precomputed values
        q_ddot         {mustBeNumeric}
        q_peak         {mustBeNumeric}
        q_dot_peak     {mustBeNumeric}
        q_ddot_to_stop {mustBeNumeric}
        q_end          {mustBeNumeric}
        % I flatten these out for simplicity, but startState is updated to
        % include these following handles too.
        % The JRS which contains the center and range to scale the
        % parameters
        paramScale rtd.util.RangeScaler = rtd.util.RangeScaler.empty()
        numParams
    end

    methods
        % The PiecewiseArmTrajectory constructor, which simply sets parameters and
        % attempts to call internalUpdate, a helper function made for this
        % class to update all other internal parameters once fully
        % parameterized.
        function self = PiecewiseArmTrajectory(startState, planTime, horizonTime, numParams)
            arguments
                startState(1,1) rtd.entity.states.ArmRobotStateInstance
                planTime(1,1) double
                horizonTime(1,1) double
                numParams(1,1) double
            end
            self.startState = startState;
            self.planTime = planTime;
            self.horizonTime = horizonTime;
            self.numParams = numParams;
        end

        function setParamScale(self, paramScale)
            arguments
                self armour.trajectory.PiecewiseArmTrajectory
                paramScale(1,1) rtd.util.RangeScaler
            end
            self.paramScale = paramScale;
        end
        
        % Set the parameters of the trajectory, with a focus on the
        % parameters as the state should be set from the constructor.
        function setParameters(self, trajectoryParams, options)
            arguments
                self armour.trajectory.PiecewiseArmTrajectory
                trajectoryParams(1,:) double
                options.startState rtd.entity.states.ArmRobotStateInstance = self.startState
                options.planTime(1,1) double = self.planTime
                options.horizonTime(1,1) double = self.horizonTime
                options.numParams(1,1) double = self.numParams
            end
            self.trajectoryParams = trajectoryParams;
            self.startState = options.startState;
            self.planTime = options.planTime;
            self.horizonTime = options.horizonTime;
            self.numParams = options.numParams;
            if length(self.trajectoryParams) > self.numParams
                self.trajectoryParams = self.trajectoryParams(1:self.numParams);
            end
            
            % Perform an internal update (compute peak and stopping values)
            self.internalUpdate();
        end
        
        % Validate that the trajectory is fully characterized
        function valid = validate(self, throwOnError)
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
        
        % Update internal parameters to reduce long term calculations.
        function internalUpdate(self)
            % internal update if valid. short circuit if not.
            if ~self.validate()
                return
            end
            
            % Rename variables
            q_0 = self.startState.position;
            q_dot_0 = self.startState.velocity;
            
            % % Scale the parameters
            % out = self.jrsInstance.output_range;
            % in = self.jrsInstance.input_range;
            % self.q_ddot = rescale(self.trajectoryParams, out(:,1), out(:,2),'InputMin',in(:,1),'InputMax',in(:,2));
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
        
        % Computes the actual input commands for the given time.
        % throws RTD:InvalidTrajectory if the trajectory isn't set
        % throws RTD:Trajectory:InvalidTime if the time is before the
        % trajectory exists
        % TODO: write in a vectorized manner
        function command = getCommand(self, time)
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
end