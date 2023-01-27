classdef BernsteinArmTrajectory < rtd.planner.trajectory.Trajectory
    % BernsteinArmTrajectory
    % This encapsulates the conversion of parameters used in optimization
    % to the actual trajectory generated from those parameters. It's also
    % indirectly used to specify the OptimizationEngine's size
    properties
        % Initial parameters from the robot used to calculate the desired
        % trajectory
        alpha   {mustBeNumeric}
        q_end   {mustBeNumeric}
        % I flatten these out for simplicity, but startState is updated to
        % include these following handles too.
        % The JRS which contains the center and range to scale the
        % parameters
        jrsInstance
    end
    methods
        % An example constructor for the trajectory object. Should be
        % implemented with varargin
        function self = BernsteinArmTrajectory(trajOptProps, startState, jrsInstance)
            arguments
                trajOptProps(1,1) rtd.planner.trajopt.TrajOptProps
                startState(1,1) rtd.entity.states.ArmRobotState
                jrsInstance(1,1) armour.reachsets.JRSInstance
            end
            self.trajOptProps = trajOptProps;
            self.startState = startState;
            self.jrsInstance = jrsInstance;
        end
        
        % A validated method to set the parameters for the trajectory.
        % Should be implemented with a varargin.
        function setParameters(self, trajectoryParams, options)
            arguments
                self armour.trajectory.BernsteinArmTrajectory
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
                self armour.trajectory.BernsteinArmTrajectory
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
            out = self.jrsInstance.output_range;
            in = self.jrsInstance.parameter_range;
            q_goal = rescale(self.trajectoryParams, out(:,1), out(:,2),'InputMin',in(:,1),'InputMax',in(:,2));

            n_q = self.jrsInstance.n_q;
            self.alpha = zeros(n_q, 6);
            for j = 1:n_q  % Modified to use matrix instead of cells
                beta = match_deg5_bernstein_coefficients({...
                    self.startState.position(j); ...
                    self.startState.velocity(j); ...
                    self.startState.acceleration(j); ...
                    q_goal(j); ...
                    0; 0});
                self.alpha(j,:) = cell2mat(bernstein_to_poly(beta, 5));
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
            horizon_mask = t_shifted < self.trajOptProps.horizonTime;
            t_masked = t_shifted(horizon_mask);
            t_masked_size = length(t_masked);
            n_q = self.jrsInstance.n_q;

            % Original implementation adapted
            q_des = zeros(n_q, t_masked_size);
            q_dot_des = zeros(n_q, t_masked_size);
            q_ddot_des = zeros(n_q, t_masked_size);
            
            for j = 1:n_q
                for coeff_idx = 0:5
                    q_des(j,:) = q_des(j,:) + ...
                        self.alpha(j,coeff_idx+1) * t_masked.^coeff_idx;
                    if coeff_idx > 0
                        q_dot_des(j,:) = q_dot_des(j,:) + ...
                            coeff_idx * self.alpha(j,coeff_idx+1) * t_masked.^(coeff_idx-1);
                    end
                    if coeff_idx > 1
                        q_ddot_des(j,:) = q_ddot_des(j,:) + ...
                            (coeff_idx) * (coeff_idx-1) * self.alpha(j,coeff_idx+1) * t_masked.^(coeff_idx-2);
                    end
                end
            end
                
            % Alternative vectorized implementation (will need updating)
            %deg = 5;
            %q_des = sum(self.alpha.*(t_masked.^(0:deg)),2);
            %q_dot_des = sum((1:deg).*self.alpha(:,2:end).*(t_masked.^(0:deg-1)),2);
            %q_ddot_des = sum(((2:deg).*(1:deg-1)).*self.alpha(:,3:end).*(t_masked.^(0:deg-2)),2);

            % Move to a combined state variable
            state = zeros(n_q*3, t_size);
            pos_idx = 1:n_q;
            vel_idx = pos_idx + n_q;
            acc_idx = vel_idx + n_q;
            state(pos_idx,horizon_mask) = q_des;
            state(vel_idx,horizon_mask) = q_dot_des;
            state(acc_idx,horizon_mask) = q_ddot_des;

            % Update all states times after the horizon time
            state(pos_idx,~horizon_mask) = repmat(self.q_end, 1, t_size-t_masked_size);
            
            % Generate the output.
            command = rtd.entity.states.ArmRobotState(pos_idx, vel_idx, acc_idx);
            command.time = time;
            command.state = state;
        end
    end
end