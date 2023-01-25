classdef PiecewiseArmTrajectory < rtd.planner.trajectory.Trajectory
    % PiecewiseArmTrajectory
    % The original ArmTD trajectory with peicewise accelerations
    properties (Constant)
        % Size of the trajectoryParams, we want this set for all use
        param_shape = 7; % This already needs a change!
        % TODO - update to add dynamic parameter as an option
    end
    properties
        % Initial parameters from the robot used to calculate the desired
        % trajectory
        q_0            {mustBeNumeric}
        q_dot_0        {mustBeNumeric}
        q_ddot_0       {mustBeNumeric}
        % Precomputed for stopping
        q_peak         {mustBeNumeric}
        q_dot_peak     {mustBeNumeric}
        q_ddot_to_stop {mustBeNumeric}
        q_end          {mustBeNumeric}
        % I flatten these out for simplicity, but startState is updated to
        % include these following handles too.
        % The JRS which contains the center and range to scale the
        % parameters
        jrsInstance
        % The starting state of the robot
        robotState
    end
    methods
        % The PiecewiseArmTrajectory constructor, which simply sets parameters and
        % attempts to call internalUpdate, a helper function made for this
        % class to update all other internal parameters once fully
        % parameterized.
        function self = PiecewiseArmTrajectory(    ...
                    trajOptProps,           ...
                    robotState,             ...
                    rsInstances,            ...
                    trajectoryParams,       ...
                    varargin                ...
                )
            % Always requires this (find way to add to parent)
            self.trajOptProps = trajOptProps;
            
            % Check for each of the args and update.
            if exist('robotState','var')
                if ~isa(robotState, 'rtd.entity.states.ArmRobotState')
                    error('Robot must inherit an rtd.entity.states.ArmRobotState to use PiecewiseArmTrajectory');
                end
                self.robotState = robotState;
            end

            if exist('rsInstances','var')
                % Look for the JRS
                for i = 1:length(rsInstances)
                    if isa(rsInstances{i}, 'armour.reachsets.JRSInstance')
                        self.jrsInstance = rsInstances{i};
                    end
                end
                if isempty(self.jrsInstance)
                    % Do something if we don't have the JRS within the
                    % passed in reachable sets
                    error('No handle for a JRSInstance was found');
                end
            end
            
            if exist('trajectoryParams','var')
                self.trajectoryParams = trajectoryParams;
            end
            
            % Perform an internal update (compute peak and stopping values)
            self.internalUpdate();
        end
        
        % Set the parameters of the trajectory, with a focus on the
        % parameters as the state should be set from the constructor.
        function setTrajectory(         ...
                    self,               ...
                    trajectoryParams,   ...
                    rsInstances,        ...
                    robotState,         ...
                    varargin            ...
                )
            % Check for each of the args and update.
            if exist('trajectoryParams','var')
                self.trajectoryParams = trajectoryParams;
            end

            if exist('rsInstances','var')
                % Look for the JRS
                for i = 1:length(rsInstances)
                    if isa(rsInstances{i}, 'JRSInstance')
                        self.jrsInstance = rsInstances{i};
                    end
                end
                if isempty(self.jrsInstance)
                    % Do something if we don't have the JRS within the
                    % passed in reachable sets
                    error('No handle for a JRSInstance was found');
                end
            end

            if exist('robotState','var')
                if ~isa(robotState, 'rtd.entity.states.ArmRobotState')
                    error('Robot must inherit an rtd.entity.states.ArmRobotState to use PiecewiseArmTrajectory');
                end
                self.robotState = robotState;
            end
            
            % Perform an internal update (compute peak and stopping values)
            % TODO: do something about the code duplication
            self.internalUpdate();
        end
        
        % Validate that the trajectory is fully characterized
        function valid = validate(self, throwOnError)
            valid = not( ...
                isempty(self.trajectoryParams) || ...
                isempty(self.jrsInstance) || ...
                isempty(self.robotState));
            
            % Throw if wanted
            if exist('throwOnError','var') && ~valid && throwOnError
                errMsg = MException('RTD:InvalidTrajectory', ...
                    'Called trajectory object does not have complete parameterization!');
                throw(errMsg)
            end
        end
        
        % Update internal parameters to reduce long term calculations.
        function internalUpdate(self)
            % internal update if valid
            if ~self.validate()
                return
            end
            
            % Set all parameters
            % Required parameters from parent classes
            self.startState.robotState = self.robotState;
            self.startState.jrsInstance = self.jrsInstance;
            
            % Parameters of our class
            self.q_0 = self.robotState.q;
            self.q_dot_0 = self.robotState.q_dot;
            self.q_ddot_0 = self.robotState.q_ddot;
            
            % Precompute peak and stop parameters
            out = self.jrsInstance.output_range;
            in = self.jrsInstance.parameter_range;
            k_scaled = rescale(self.trajectoryParams, out(:,1), out(:,2),'InputMin',in(:,1),'InputMax',in(:,2));
            
            self.q_peak = self.q_0 + ...
                self.q_dot_0 * self.trajOptProps.planTime + ...
                (1/2) * k_scaled * self.trajOptProps.planTime^2;
            self.q_dot_peak = self.q_dot_0 + ...
                k_scaled * self.trajOptProps.planTime;
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
            % Validate, if invalid, throw
            self.validate(true);
            % TODO: throw invalid trajectory
            t = time - self.robotState.time;
            
            out = self.jrsInstance.output_range;
            in = self.jrsInstance.parameter_range;
            k_scaled = rescale(self.trajectoryParams, out(:,1), out(:,2),'InputMin',in(:,1),'InputMax',in(:,2));
            
            % Ensure time is valid
            if t < 0
                ME = MException('RTD:Trajectory:InvalidTime', ...
                    'Invalid time provided to PiecewiseArmTrajectory');
                throw(ME)
                
            % First half of the trajectory
            elseif t < self.trajOptProps.planTime
                q_des = self.q_0 + ...
                    self.q_dot_0 * t + ...
                    (1/2) * k_scaled * t^2;
                q_dot_des = self.q_dot_0 + ...
                    k_scaled * t;
                q_ddot_des = k_scaled;
            
            % Second half of the trajectory
            elseif t < self.trajOptProps.horizonTime
                % Shift time for ease
                t = t - self.trajOptProps.planTime;
                
                q_des = self.q_peak + ...
                    self.q_dot_peak * t + ...
                    (1/2) * self.q_ddot_to_stop * t^2;
                q_dot_des = self.q_dot_peak + ...
                    self.q_ddot_to_stop * t;
                q_ddot_des = self.q_ddot_to_stop;
            
            % The trajectory has reached a stop
            else
                q_des = self.q_end;
                q_dot_des = zeros(size(self.q_dot_0));
                q_ddot_des = zeros(size(self.q_ddot_0));
            end

            n_q = length(q_des);
            command = rtd.entity.states.ArmRobotState(1:n_q, n_q+1:n_q*2, n_q*2+1:n_q*3);
            command.time = time;
            command.state = [q_des; q_dot_des; q_ddot_des];
        end
    end
end