classdef BernsteinArmTrajectory < rtd.trajectory.Trajectory
    % BernsteinArmTrajectory
    % This encapsulates the conversion of parameters used in optimization
    % to the actual trajectory generated from those parameters. It's also
    % indirectly used to specify the OptimizationEngine's size
    % Required properties
    properties (Constant)
        vectorized = true
    end
    % Additional properties
    properties
        startState(1,1) rtd.entity.states.ArmRobotStateInstance
        horizonTime

        % Initial parameters from the robot used to calculate the desired
        % trajectory
        alpha   {mustBeNumeric}
        q_end   {mustBeNumeric}

        % Extra properties to scale the parameters
        paramScale rtd.util.RangeScaler = rtd.util.RangeScaler.empty()
        numParams
    end
    methods
        % An example constructor for the trajectory object. Should be
        % implemented with varargin
        function self = BernsteinArmTrajectory(startState, horizonTime, numParams)
            arguments
                startState(1,1) rtd.entity.states.ArmRobotStateInstance
                horizonTime(1,1) double
                numParams(1,1) double
            end
            
            self.startState = startState;
            self.horizonTime = horizonTime;
            self.numParams = numParams;
        end

        function setParamScale(self, paramScale)
            arguments
                self armour.trajectory.BernsteinArmTrajectory
                paramScale(1,1) rtd.util.RangeScaler
            end
            self.paramScale = paramScale;
        end
        
        % A validated method to set the parameters for the trajectory.
        % Should be implemented with a varargin.
        function setParameters(self, trajectoryParams, options)
            arguments
                self armour.trajectory.BernsteinArmTrajectory
                trajectoryParams(1,:) double
                options.startState rtd.entity.states.ArmRobotStateInstance = self.startState
                options.horizonTime(1,1) double = self.horizonTime
                options.numParams(1,1) double = self.numParams
            end
            self.trajectoryParams = trajectoryParams;
            self.startState = options.startState;
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
                self armour.trajectory.BernsteinArmTrajectory
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
                errMsg = MException('BernsteinArmTrajectory:InvalidTrajectory', ...
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
            
            % Get the desired final position
            if isempty(self.paramScale)
                q_goal = self.trajectoryParams;
            else
                q_goal = self.startState.position + self.paramScale.scaleout(self.trajectoryParams);
            end

            self.alpha = zeros(self.numParams, 6);
            for j = 1:self.numParams  % Modified to use matrix instead of cells
                beta = armour.legacy.match_deg5_bernstein_coefficients({...
                    self.startState.position(j); ...
                    self.startState.velocity(j); ...
                    self.startState.acceleration(j); ...
                    q_goal(j); ...
                    0; 0}, self.horizonTime);
                self.alpha(j,:) = cell2mat(armour.legacy.bernstein_to_poly(beta, 5));
            end
            
            % Precompute end position
            % Adapted Original
            % End position should actually just be q_goal
            %self.q_end = zeros(self.n_q, 1);
            %for j = 1:n_q
            %    for coeff_idx = 0:5
            %        self.q_end(j) = self.q_end(j) + ...
            %            self.alpha(j,coeff_idx+1) * self.trajOptProps.horizonTime^coeff_idx;
            %    end
            %end
            self.q_end = q_goal;
            % Proposed vectorized implementation
            %self.q_end = sum(self.alpha.*(trajOptProps.horizon.^(0:5)),2);
        end
        
        % Computes the actual input commands for the given time.
        % throws RTD:InvalidTrajectory if the trajectory isn't set
        % TODO: write in a vectorized manner
        function command = getCommand(self, time)
            arguments
                self armour.trajectory.BernsteinArmTrajectory
                time(1,:) double
            end

            % Do a parameter check and time check, and throw if anything is
            % invalid.
            self.validate(true);
            t_shifted = time - self.startState.time;
            if any(t_shifted < 0)
                ME = MException('BernsteinArmTrajectory:InvalidTime', ...
                    'Invalid time provided to BernsteinArmTrajectory');
                throw(ME)
            end
            
            % Get a mask for the active trajectory time and the stopped
            % trajectory times.
            t_size = length(t_shifted);
            horizon_mask = t_shifted < self.horizonTime;
            t_masked_scaled = t_shifted(horizon_mask) / self.horizonTime;
            t_masked_size = length(t_masked_scaled);

            % Original implementation adapted
            q_des = zeros(self.numParams, t_masked_size);
            q_dot_des = zeros(self.numParams, t_masked_size);
            q_ddot_des = zeros(self.numParams, t_masked_size);
            
            for j = 1:self.numParams
                for coeff_idx = 0:5
                    q_des(j,:) = q_des(j,:) + ...
                        self.alpha(j,coeff_idx+1) * t_masked_scaled.^coeff_idx;
                    if coeff_idx > 0
                        q_dot_des(j,:) = q_dot_des(j,:) + ...
                            coeff_idx * self.alpha(j,coeff_idx+1) * t_masked_scaled.^(coeff_idx-1);
                    end
                    if coeff_idx > 1
                        q_ddot_des(j,:) = q_ddot_des(j,:) + ...
                            (coeff_idx) * (coeff_idx-1) * self.alpha(j,coeff_idx+1) * t_masked_scaled.^(coeff_idx-2);
                    end
                end
            end
                
            % Alternative vectorized implementation (will need updating)
            %deg = 5;
            %q_des = sum(self.alpha.*(t_masked.^(0:deg)),2);
            %q_dot_des = sum((1:deg).*self.alpha(:,2:end).*(t_masked.^(0:deg-1)),2);
            %q_ddot_des = sum(((2:deg).*(1:deg-1)).*self.alpha(:,3:end).*(t_masked.^(0:deg-2)),2);

            % Move to a combined state variable
            state = zeros(self.numParams*3, t_size);
            pos_idx = 1:self.numParams;
            vel_idx = pos_idx + self.numParams;
            acc_idx = vel_idx + self.numParams;
            state(pos_idx,horizon_mask) = q_des;
            state(vel_idx,horizon_mask) = q_dot_des / self.horizonTime;
            state(acc_idx,horizon_mask) = q_ddot_des / self.horizonTime^2;

            % Update all states times after the horizon time
            state(pos_idx,~horizon_mask) = repmat(self.q_end, 1, t_size-t_masked_size);
            
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