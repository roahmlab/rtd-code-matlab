classdef PiecewiseArmTrajectory < rtd.planner.trajectory.Trajectory
    % PiecewiseArmTrajectory
    % The original ArmTD trajectory with peicewise accelerations
    properties
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
        jrsInstance
    end

    methods
        % The PiecewiseArmTrajectory constructor, which simply sets parameters and
        % attempts to call internalUpdate, a helper function made for this
        % class to update all other internal parameters once fully
        % parameterized.
        function self = PiecewiseArmTrajectory(trajOptProps, startState, jrsInstance)
            arguments
                trajOptProps(1,1) rtd.planner.trajopt.TrajOptProps
                startState(1,1) rtd.entity.states.ArmRobotState
                jrsInstance(1,1) armour.reachsets.JRSInstance
            end
            self.trajOptProps = trajOptProps;
            self.startState = startState;
            self.jrsInstance = jrsInstance;
        end
        
        % Set the parameters of the trajectory, with a focus on the
        % parameters as the state should be set from the constructor.
        function setParameters(self, trajectoryParams, options)
            arguments
                self armour.trajectory.PiecewiseArmTrajectory
                trajectoryParams(1,:) double
                options.startState rtd.entity.states.ArmRobotState = self.startState
                options.jrsInstance armour.reachsets.JRSInstance = self.jrsInstance
            end
            self.trajectoryParams = trajectoryParams;
            self.startState = options.startState;
            self.jrsInstance = options.jrsInstance;
            
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
                isempty(self.jrsInstance) || ...
                isempty(self.startState));

            % Make sure the trajectory params make sense
            valid = valid && length(self.trajectoryParams) == self.jrsInstance.n_q;
            
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
            
            % Scale the parameters
            out = self.jrsInstance.output_range;
            in = self.jrsInstance.parameter_range;
            self.q_ddot = rescale(self.trajectoryParams, out(:,1), out(:,2),'InputMin',in(:,1),'InputMax',in(:,2));
            
            % Compute the peak parameters
            self.q_peak = q_0 + ...
                q_dot_0 * self.trajOptProps.planTime + ...
                (1/2) * self.q_ddot * self.trajOptProps.planTime^2;
            self.q_dot_peak = q_dot_0 + ...
                self.q_ddot * self.trajOptProps.planTime;

            % Compute the stopping parameters
            self.q_ddot_to_stop = (0-self.q_dot_peak) / ...
                (self.trajOptProps.horizonTime - self.trajOptProps.planTime);
            self.q_end = self.q_peak + ...
                self.q_dot_peak * self.trajOptProps.planTime + ...
                (1/2) * self.q_ddot_to_stop * self.trajOptProps.planTime^2;
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
            t_shifted = time - self.startState.time;
            if t_shifted < 0
                ME = MException('PiecewiseArmTrajectory:InvalidTrajectory', ...
                    'Invalid time provided to PiecewiseArmTrajectory');
                throw(ME)
            end

            % Mask the first and second half of the trajectory
            t_plan_mask = t_shifted < self.trajOptProps.planTime;
            t_stop_mask = logical((t_shifted < self.trajOptProps.horizonTime) - t_plan_mask);
            t_plan_vals = t_shifted(t_plan_mask);
            t_stop_vals = t_shifted(t_stop_mask) - self.trajOptProps.planTime;

            % Create the combined state variable
            t_size = length(time);
            n_q = self.jrsInstance.n_q;
            pos_idx = 1:n_q;
            vel_idx = pos_idx + n_q;
            acc_idx = vel_idx + n_q;
            state = zeros(n_q*3, t_size);
            
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
            command = rtd.entity.states.ArmRobotState(pos_idx, vel_idx, acc_idx);
            command.time = time;
            command.state = state;
        end
    end
end