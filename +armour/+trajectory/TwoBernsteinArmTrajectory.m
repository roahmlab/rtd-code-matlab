classdef TwoBernsteinArmTrajectory < rtd.trajectory.Trajectory
% Trajectory utilizing two 5th Degree Bernstein Polynomials for arm robots
%
% This trajectory uses two 5th degree Bernstein polynomials to generate
% a trajectory for arm robots. It is parameterized by the start state,
% the plan time, the horizon time, and the desired final position. The
% major bernstein polynomial is used to generate the trajectory from the
% start state to the plan time, but is characterized by the horizon time.
% The minor bernstein polynomial is used to generate the trajectory from
% the plan time to the horizon time and takes the place of the final
% position from the major bernstein polynomial.
%
% The trajectory is parameterized by the following:
%  * `startState`: The start state of the trajectory. This is used to
%    specify the initial position, velocity, and acceleration of the trajectory.
%  * `planTime`: The time at which the major trajectory should stop being executed, and
%    the minor trajectory should start being executed. This is used to
%    specify the active duration of the major trajectory.
%  * `horizonTime`: The time at which the overall trajectory should end. This is
%    used to specify the duration of the trajectory.
%  * `params`: The desired final position of each polynomial relative to (1) the start
%    position for the major polynomial and (2) the position at planTime for the
%    minor polynomial. The relative position can also be scaled, which then
%    paramScale is used to scale it back to the desired position.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-05-17
% Updated: 2023-10-04 (Adam Li)
%
% See also: rtd.trajectory.Trajectory, armour.trajectory.ArmTrajectoryFactory,
% rtd.entity.states.ArmRobotStateInstance, armour.trajectory.BernsteinArmTrajectory
%
% --- More Info ---
%

    % Required properties
    properties (Constant)
        % This does not support vectorized calls
        vectorized = false
    end

    % Additional properties
    properties
        % The initial state of the trajectory. These are initial parameters
        % from the robot used to calculate the desired bernstein coefficients
        startState(1,1) rtd.entity.states.ArmRobotStateInstance

        % The duration of the trajectory, after which the trajectory will
        % hold the final position
        planTime(1,1) double

        % The duration of the trajectory, after which the trajectory will
        % hold the final position
        horizonTime(1,1) double

        % Extra properties to scale the parameters
        paramScale rtd.util.RangeScaler = rtd.util.RangeScaler.empty()

        % The number of parameters for the trajectory
        numParams(1,1) uint32
    end

    % Internal properties
    properties (SetAccess=private)
        % the alpha coefficients for the major bernstein polynomial
        alpha(:,6) double

        % the alpha coefficients for the minor bernstein polynomial
        alpha2(:,6) double

        % The calculated final position of the trajectory
        q_end(:,1) double

        % The position where the minor polynomial takes over
        q_plan(:,1) double

        % The velocity where the minor polynomial takes over
        qd_plan(:,1) double
        
        % The acceleration where the minor polynomial takes over
        qdd_plan(:,1) double
    end

    methods
        function self = TwoBernsteinArmTrajectory(startState, planTime, horizonTime, numParams)
            % Constructor for the BernsteinArmTrajectory
            %
            % Arguments:
            %   startState: The start state of the trajectory. This is used to
            %       specify the initial position, velocity, and acceleration of the trajectory.
            %   planTime: The time at which the major trajectory should stop being executed, and
            %       the minor trajectory should start being executed. This is used to
            %       specify the active duration of the major polynomial.
            %   horizonTime: The time at which the overall trajectory should end. This is
            %       used to specify the duration of the overall trajectory.
            %   numParams: The number of parameters for the trajectory. This is
            %       used to specify the number of parameters for the trajectory.
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
                self armour.trajectory.TwoBernsteinArmTrajectory
                paramScale(:,1) rtd.util.RangeScaler = []
            end

            if length(paramScale) > 1
                error('Can only set one paramScale value!');
            end
            self.paramScale = paramScale;
        end
        
        function setParameters(self, trajectoryParams, options)
            % Sets the parameters for the trajectory.
            %
            % After setting the parameters, it attempts to internally update the trajectory
            % to reduce long term calculations.
            %
            % Arguments:
            %   trajectoryParams (double): The parameters for the trajectory. This is
            %       used to specify the relative position of the trajectory.
            %       The relative position can also be scaled, which then
            %       paramScale is used to scale it back to the desired position.
            %   options: Keyword arguments. See Below.
            %
            % Keyword Arguments:
            %   startState (rtd.entity.states.ArmRobotStateInstance): The start state of the trajectory. This is used to
            %       specify the initial position, velocity, and acceleration of the trajectory.
            %   planTime (double): The time at which the major trajectory should stop being executed, and
            %       the minor trajectory should start being executed. This is used to
            %       specify the active duration of the major polynomial.
            %   horizonTime (double): The time at which the overall trajectory should end. This is
            %       used to specify the duration of the overall trajectory.
            %   numParams (double): The number of parameters for the trajectory. This is
            %       used to specify the number of parameters for the trajectory.
            %   paramScale (rtd.util.RangeScaler): The parameter scaling to use.
            %       This is used to scale the relative position to the desired
            %       position.
            %
            arguments
                self armour.trajectory.TwoBernsteinArmTrajectory
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
            %   valid (logical): Whether or not the trajectory parameters are valid.
            %
            % Raises:
            %   InvalidTrajectory: If the current trajectory is not valid and
            %       throwOnError is true.
            %
            arguments
                self armour.trajectory.TwoBernsteinArmTrajectory
                throwOnError(1,1) logical = false
            end

            % Make sure everything is nonempty
            valid = not( ...
                isempty(self.trajectoryParams) || ...
                isempty(self.startState));

            % Make sure the trajectory params make sense
            valid = valid && length(self.trajectoryParams) == self.numParams;
            valid = valid && self.numParams/2 == length(self.startState.position);
            valid = valid && self.planTime < self.horizonTime;

            % Throw if wanted
            if ~valid && throwOnError
                errMsg = MException('TwoBernsteinArmTrajectory:InvalidTrajectory', ...
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
                self armour.trajectory.TwoBernsteinArmTrajectory
                time(1,:) double
            end

            % Do a parameter check and time check, and throw if anything is
            % invalid.
            self.validate(true);
            t_shifted = time - self.startTime;
            if any(t_shifted < 0)
                ME = MException('TwoBernsteinArmTrajectory:InvalidTime', ...
                    'Invalid time provided to TwoBernsteinArmTrajectory');
                throw(ME)
            end
            
            % Get a mask for the active trajectory time and the stopped
            % trajectory times.
            % t_size = length(t_shifted);
            % horizon_mask = t_shifted < self.horizonTime;
            % t_masked_scaled = t_shifted(horizon_mask) / self.horizonTime;
            % t_masked_size = length(t_masked_scaled);

            % Original implementation adapted
            q_des = zeros(self.numParams/2, 1);
            q_dot_des = zeros(self.numParams/2, 1);
            q_ddot_des = zeros(self.numParams/2, 1);

            if t_shifted < self.planTime
                scale_coeff = 1.0 / self.horizonTime;
                t_scaled = t_shifted * scale_coeff;
                alpha = self.alpha;
            elseif t_shifted < self.horizonTime
                scale_coeff = 1.0 / (self.horizonTime - self.planTime);
                t_scaled = (t_shifted - self.planTime) * scale_coeff;
                alpha = self.alpha2;
            end

            if t_shifted < self.horizonTime
                for j = 1:self.numParams/2
                    for coeff_idx = 0:5
                        q_des(j,:) = q_des(j,:) + ...
                            alpha(j,coeff_idx+1) * t_scaled.^coeff_idx;
                        if coeff_idx > 0
                            q_dot_des(j,:) = q_dot_des(j,:) + ...
                                coeff_idx * alpha(j,coeff_idx+1) * t_scaled.^(coeff_idx-1);
                        end
                        if coeff_idx > 1
                            q_ddot_des(j,:) = q_ddot_des(j,:) + ...
                                (coeff_idx) * (coeff_idx-1) * alpha(j,coeff_idx+1) * t_scaled.^(coeff_idx-2);
                        end
                    end
                end
                q_dot_des = q_dot_des * scale_coeff;
                q_ddot_des = q_ddot_des * scale_coeff^2;
            else
                q_des = self.q_end;
            end
            
            % Generate the output.
            command = rtd.entity.states.ArmRobotStateInstance();
            command.setTimes(time);
            command.position = q_des;
            command.velocity = q_dot_des;
            command.acceleration = q_ddot_des;
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
            
            % Get the desired final position & scale parameters as needed
            n_q = length(self.startState.position);
            if isempty(self.paramScale)
                scaled_params = self.trajectoryParams; 
            else
                scaled_params = self.paramScale.scaleout(self.trajectoryParams);
            end
            q_goal = self.startState.position + scaled_params(1:self.numParams/2);

            % Get major parameters
            self.alpha = zeros(self.numParams/2, 6);
            for j = 1:self.numParams/2  % Modified to use matrix instead of cells
                beta = armour.legacy.match_deg5_bernstein_coefficients({...
                    self.startState.position(j); ...
                    self.startState.velocity(j); ...
                    self.startState.acceleration(j); ...
                    q_goal(j); ...
                    0; 0}, self.horizonTime);
                self.alpha(j,:) = cell2mat(armour.legacy.bernstein_to_poly(beta, 5));
            end

            % Compute middle position
            q_plan = zeros(n_q, 1);
            qd_plan = zeros(n_q, 1);
            qdd_plan = zeros(n_q, 1);
            for j = 1:n_q
                for coeff_idx = 0:5
                    q_plan(j,:) = q_plan(j,:) + ...
                        self.alpha(j,coeff_idx+1) * (self.planTime).^coeff_idx;
                    if coeff_idx > 0
                        qd_plan(j,:) = qd_plan(j,:) + ...
                            coeff_idx * self.alpha(j,coeff_idx+1) * (self.planTime).^(coeff_idx-1);
                    end
                    if coeff_idx > 1
                        qdd_plan(j,:) = qdd_plan(j,:) + ...
                            (coeff_idx) * (coeff_idx-1) * self.alpha(j,coeff_idx+1) * (self.planTime).^(coeff_idx-2);
                    end
                end
            end
            self.q_plan = q_plan;
            self.qd_plan = qd_plan/(self.horizonTime);
            self.qdd_plan = qdd_plan/(self.horizonTime)^2;
            

            if ~isempty(self.paramScale)
                q_goal2 = q_plan + scaled_params((self.numParams/2+1):end);
            end

            % Get minor parameters
            self.alpha2 = zeros(self.numParams/2, 6);
            for j = 1:self.numParams/2  % Modified to use matrix instead of cells
                beta = armour.legacy.match_deg5_bernstein_coefficients({...
                    self.q_plan(j); ...
                    self.qd_plan(j); ...
                    self.qdd_plan(j); ...
                    q_goal2(j); ...
                    0; 0}, self.horizonTime - self.planTime);
                self.alpha2(j,:) = cell2mat(armour.legacy.bernstein_to_poly(beta, 5));
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
            
            self.q_end = q_goal2;
            self.startTime = self.startState.time;
        end
    end
end