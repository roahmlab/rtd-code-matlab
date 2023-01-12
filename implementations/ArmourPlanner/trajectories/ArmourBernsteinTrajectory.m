classdef ArmourBernsteinTrajectory < Trajectory
    % ArmourBernsteinTrajectory
    % This encapsulates the conversion of parameters used in optimization
    % to the actual trajectory generated from those parameters. It's also
    % indirectly used to specify the OptimizationEngine's size
    properties (Constant)
        % Size of the trajectoryParams, we want this set for all use
        param_shape = 7; % This already needs a change!
    end
    properties
        % Initial parameters from the robot used to calculate the desired
        % trajectory
        n_q     {mustBeNumeric, mustBeScalarOrEmpty}
        alpha   {mustBeNumeric}
        q_end   {mustBeNumeric}
        % I flatten these out for simplicity, but startState is updated to
        % include these following handles too.
        % The JRS which contains the center and range to scale the
        % parameters
        jrsInstance
        % The starting state of the robot
        robotState
    end
    methods
        % An example constructor for the trajectory object. Should be
        % implemented with varargin
        function self = ArmourBernsteinTrajectory(     ...
                    trajOptProps,       ...
                    robotState,         ...
                    rsInstances,        ...
                    trajectoryParams,   ...
                    varargin            ...
                )
            % Always requires this (find way to add to parent)
            self.trajOptProps = trajOptProps;
            
            % Check for each of the args and update.
            if exist('robotState','var')
                if ~isa(robotState, 'ArmRobotTrajectoryState')
                    error('Robot must inherit an ArmRobotTrajectoryState to use ArmTdTrajectory');
                end
                self.robotState = robotState;
            end
            
            % Look for the JRS
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

            % Set parameters
            if exist('trajectoryParams','var')
                self.trajectoryParams = trajectoryParams;
            end
            
            % Perform an internal update (compute peak and stopping values)
            self.internalUpdate();
        end
        
        % A validated method to set the parameters for the trajectory.
        % Should be implemented with a varargin.
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
                if ~isa(robotState, 'ArmRobotState')
                    error('Robot must inherit an ArmRobotState to use ArmTdTrajectory');
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
            out = self.jrsInstance.output_range;
            in = self.jrsInstance.parameter_range;
            q_goal = rescale(self.trajectoryParams, out(:,1), out(:,2),'InputMin',in(:,1),'InputMax',in(:,2));
            %q_goal = self.jrsInstance.jrs_info.c_k_bernstein + self.jrsInstance.jrs_info.g_k_bernstein.*self.trajectoryParams;
            self.n_q = length(self.robotState.q);
            self.alpha = zeros(self.n_q, 6);
            for j = 1:self.n_q  % Modified to use matrix instead of cells
                beta = match_deg5_bernstein_coefficients({...
                    self.robotState.q(j); ...
                    self.robotState.q_dot(j); ...
                    self.robotState.q_ddot(j); ...
                    q_goal(j); ...
                    0; 0});
                self.alpha(j,:) = cell2mat(bernstein_to_poly(beta, 5));
            end
            
            % Precompute end position
            % Adapted Original
            % End position should actually just be q_goal
            %self.q_end = zeros(self.n_q, 1);
            %for j = 1:self.n_q
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
            % TODO: throw invalid trajectory
            t = time - self.robotState.time;
            
            % Ensure time is valid
            if t < 0
                ME = MException('RTD:Trajectory:InvalidTime', ...
                    'Invalid time provided to ArmourBernsteinTrajectory');
                throw(ME)
            
            % Valid time of trajectory
            elseif t < self.trajOptProps.horizonTime
                % Original implementation adapted
                q_des = zeros(self.n_q, 1);
                q_dot_des = zeros(self.n_q, 1);
                q_ddot_des = zeros(self.n_q, 1);
                % TODO: Figure out how to adapt this!!!
            
                for j = 1:self.n_q
                    for coeff_idx = 0:5
                        q_des(j) = q_des(j) + ...
                            self.alpha(j,coeff_idx+1) * t^coeff_idx;
                        if coeff_idx > 0
                            q_dot_des(j) = q_dot_des(j) + ...
                                coeff_idx * self.alpha(j,coeff_idx+1) * t^(coeff_idx-1);
                        end
                        if coeff_idx > 1
                            q_ddot_des(j) = q_ddot_des(j) + ...
                                (coeff_idx) * (coeff_idx-1) * self.alpha(j,coeff_idx+1) * t^(coeff_idx-2);
                        end
                    end
                end
                
                % Proposed vectorized implementation
                %deg = 5;
                %command.q_des = sum(self.alpha.*(t.^(0:deg)),2);
                %command.q_dot_des = sum((1:deg).*self.alpha(:,2:end).*(t.^(0:deg-1)),2);
                %command.q_ddot_des = sum(((2:deg).*(1:deg-1)).*self.alpha(:,3:end).*(t.^(0:deg-2)),2);

            % The trajectory has reached a stop
            else
                q_des = self.q_end;
                q_dot_des = zeros(self.n_q, 1);
                q_ddot_des = zeros(self.n_q, 1);
            end
            
            command = ArmRobotTrajectoryState(1:self.n_q, self.n_q+1:self.n_q*2, self.n_q*2+1:self.n_q*3);
            command.time = time;
            command.state = [q_des; q_dot_des; q_ddot_des];
        end
    end
end