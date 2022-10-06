classdef ArmTdTrajectory < Trajectory
    % ArmTdTrajectory
    % This encapsulates the conversion of parameters used in optimization
    % to the actual trajectory generated from those parameters. It's also
    % indirectly used to specify the OptimizationEngine's size
    properties (Constant)
        % Size of the trajectoryParams, we want this set for all use
        param_shape = 7; % This already needs a change!
    end
    properties
        % The JRS which contains the center and range to scale the
        % parameters
        jrsInstance
        % Initial parameters from the robot used to calculate the desired
        % trajectory
        q_0         {mustBeNumeric}
        q_dot_0     {mustBeNumeric}
        q_ddot_0    {mustBeNumeric}
        % Precomputed for stopping
        q_peak      {mustBeNumeric}
        q_dot_peak  {mustBeNumeric}
        q_ddot_stop {mustBeNumeric}
        q_end      {mustBeNumeric}
        % The parameters of the trajopt instance (move to parent?)
        trajOptProps
        robotState
    end
    methods
        % An example constructor for the trajectory object. Should be
        % implemented with varargin
        function self = ArmTdTrajectory(     ...
                    trajOptProps,         ...
                    robotState,         ...
                    rsInstances,      ...
                    trajectoryParams,   ...
                    varargin            ...
                )
            % Always requires this (add to parent)
            self.trajOptProps = trajOptProps;
            
            % If none of the args exist, just create the empty object
            if ~exist('robotState','var')
                return
            end
            
            % Validate RobotState
            if ~isa(robotState, 'ArmRobotState')
                error('Robot must inherit an ArmRobotState to use ArmTdTrajectory');
            end
            self.robotState = robotState;
            
            % Clean up, but if reachableSet is nonexistant, end (invert
            % logic later)
            if ~exist('rsInstances','var')
                return
            end
            
            % Look for the JRS
            for i = 1:length(rsInstances)
                if isa(rsInstances{i}, 'JRSInstance')
                    self.jrsInstance = rsInstances{i};
                end
            end
            if isempty(self.jrsInstance)
                % Do something if we don't have the JRS
                error('No handle for a JRSInstance was found');
            end
            
            % Clean up, but if reachableSet is nonexistant, end (invert
            % logic later)
            if ~exist('trajectoryParams','var')
                return
            end
            self.trajectoryParams = trajectoryParams;

            
            % TODO: Make invalid trajectory case.
            self.internalUpdate();
        end
        
        % A validated method to set the parameters for the trajectory.
        % Should be implemented with a varargin.
        function setTrajectory(         ...
                    self,               ...
                    trajectoryParams,   ...
                    rsInstances,      ...
                    robotState          ...
                )
            % TODO: make
            % Clean up, but if reachableSet is nonexistant, end (invert
            % logic later)
            if ~exist('trajectoryParams','var')
                try
                    self.internalUpdate();
                catch
                end
                return
            end
            self.trajectoryParams = trajectoryParams;
            
            % Clean up, but if reachableSet is nonexistant, end (invert
            % logic later)
            if ~exist('rsInstances','var')
                try
                    self.internalUpdate();
                catch
                end
                return
            end
            
            % Look for the JRS
            for i = 1:length(rsInstances)
                if isa(rsInstances{i}, 'JRSInstance')
                    self.jrsInstance = rsInstances{i};
                end
            end
            if isempty(self.jrsInstance)
                % Do something if we don't have the JRS
                error('No handle for a JRSInstance was found');
            end
            
            % If none of the args exist, just create the empty object
            if ~exist('robotState','var')
                try
                    self.internalUpdate();
                catch
                end
                return
            end
            
            % Validate RobotState
            if ~isa(robotState, 'ArmRobotState')
                error('Robot must inherit an ArmRobotState to use ArmTdTrajectory');
            end
            self.robotState = robotState;
            
            self.internalUpdate();
        end
        
        function internalUpdate(self)
            % internal update if valid
            if ~self.validate()
                return
            end
            
            % Set all parameters
            % Required parameters from parent classes
            self.startTime = self.robotState.time;
            
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
            self.q_ddot_stop = (0-self.q_dot_peak) / ...
                (self.trajOptProps.horizonTime - self.trajOptProps.planTime);
            self.q_end = self.q_peak + ...
                self.q_dot_peak * self.trajOptProps.horizonTime + ...
                (1/2) * self.q_ddot_stop * self.trajOptProps.horizonTime^2;
        end
        
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
            
        
        % A validated method to get the trajectory parameters.
        % throws RTD:InvalidTrajectory if there isn't a trajectory to get.
        function [trajectoryParams, startState] = getTrajParams(self)
            % Validate, if invalid, throw
            self.validate(true);
            % TODO: make into the base class!
            trajectoryParams = self.trajectoryParams;
            startState.robotState = self.robotState;
            startState.jrsInstance = self.jrsInstance;
        end
        
        % Computes the actual input commands for the given time.
        % throws RTD:InvalidTrajectory if the trajectory isn't set
        function command = getCommand(self, time)
            % Validate, if invalid, throw
            self.validate(true);
            % TODO: throw invalid trajectory
            t = time - self.startTime;
            
            out = self.jrsInstance.output_range;
            in = self.jrsInstance.parameter_range;
            k_scaled = rescale(self.trajectoryParams, out(:,1), out(:,2),'InputMin',in(:,1),'InputMax',in(:,2));
            
            % Ensure time is valid
            if t < 0
                ME = MException('RTD:Trajectory:InvalidTime', ...
                    'Invalid time provided to ArmTdTrajectory');
                throw(ME)
                
            % First half of the trajectory
            elseif t < self.trajOptProps.planTime
                command.q_des = self.q_0 + ...
                    self.q_dot_0 * t + ...
                    (1/2) * k_scaled * t^2;
                command.q_dot_des = self.q_dot_0 + ...
                    k_scaled * t;
                command.q_ddot_des = k_scaled;
            
            % Second half of the trajectory
            elseif t <= self.trajOptProps.horizonTime
                % Shift time for ease
                t = t - self.trajOptProps.planTime;
                
                command.q_des = self.q_peak + ...
                    self.q_dot_peak * t + ...
                    (1/2) * self.q_ddot_stop * t^2;
                command.q_dot_des = self.q_dot_peak + ...
                    self.q_ddot_stop * t;
                command.q_ddot_des = self.q_ddot_stop;
            
            % The trajectory has reached a stop
            else
                command.q_des = self.q_end;
                command.q_dot_des = zeros(size(self.q_dot_0));
                command.q_ddot_des = zeros(size(self.q_ddot_0));
            end
        end
    end
end