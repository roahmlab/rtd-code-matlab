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
        % The JRS which contains the center and range to scale the
        % parameters
        jrsInstance
        % Initial parameters from the robot used to calculate the desired
        % trajectory
        n_q     {mustBeNumeric, mustBeScalarOrEmpty}
        alpha   {mustBeNumeric}
        q_end   {mustBeNumeric}
        % The parameters of the trajopt instance (move to parent?)
        trajOptProps
    end
    methods
        % An example constructor for the trajectory object. Should be
        % implemented with varargin
        function self = ArmourBernsteinTrajectory(     ...
                    trajOptProps,         ...
                    robotState,         ...
                    reachableSets,      ...
                    trajectoryParams,   ...
                    varargin            ...
                )
            % Validate RobotState
            if ~isa(robotState, 'ArmRobotState')
                error('Robot must inherit an ArmRobotState to use ArmTdTrajectory');
            end
            
            % Look for the JRS
            for i = 1:length(reachableSets)
                if isa(reachableSets{i}, 'BernsteinJRSInstance')
                    self.jrsInstance = reachableSets{i};
                end
            end
            if isempty(self.jrsInstance)
                % Do something if we don't have the JRS
            end
            
            % Set all parameters
            % Required parameters from parent classes
            self.trajectoryParams = trajectoryParams;
            self.startTime = robotState.time;
            
            % Parameters of our class
            self.q_goal = self.jrsInstance.c_k_bernstein + P.jrs_info.g_k_bernstein.*trajectoryParams;
            % Propose change name to k_center_bernstein or k_c_bernstein
            % and k_range_bernstein or k_g_bernstein?
            self.n_q = length(robotState.q);
            self.alpha = zeros(self.n_q, 6);
            for j = 1:self.n_q  % Modified to use matrix instead of cells
                beta = match_deg5_bernstein_coefficients({...
                    robotState.q(j); ...
                    robotState.q_dot(j); ...
                    robotState.q_ddot(j); ...
                    self.q_goal(j); ...
                    0; 0});
                self.alpha(j,:) = cell2mat(bernstein_to_poly(beta, 5));
            end
            self.trajOptProps = trajOptProps;
            
            % Precompute end position
            % Adapted Original
            self.q_end = zeros(self.n_q, 1);
            for j = 1:self.n_q
                for coeff_idx = 0:5
                    self.q_end(j) = self.q_end(j) + ...
                        self.alpha(j,coeff_idx+1) * trajOptProps.horizon^coeff_idx;
                end
            end
            % Proposed vectorized implementation
            %self.q_end = sum(self.alpha.*(trajOptProps.horizon.^(0:5)),2);
            
            % TODO: Make invalid trajectory case.
        end
        
        % A validated method to set the parameters for the trajectory.
        % Should be implemented with a varargin.
        function setTrajectory(         ...
                    self,               ...
                    trajectoryParams,   ...
                    reachableSets,      ...
                    robotState          ...
                )
            % TODO: make
        end
        
        % A validated method to get the trajectory parameters.
        % throws RTD:InvalidTrajectory if there isn't a trajectory to get.
        function [trajectoryParams, startTime] = getTrajParams(self)
            % TODO: make into the base class!
        end
        
        % Computes the actual input commands for the given time.
        % throws RTD:InvalidTrajectory if the trajectory isn't set
        function command = getCommand(self, time)
            % TODO: throw invalid trajectory
            t = time - startTime;
            
            % Ensure time is valid
            if t < 0
                ME = MException('RTD:Trajectory:InvalidTime', ...
                    'Invalid time provided to ArmourBernsteinTrajectory');
                throw(ME)
            
            % Valid time of trajectory
            elseif t < self.trajOptProps.horizon
                % Original implementation adapted
                command.q_des = zeros(self.n_q, 1);
                command.q_dot_des = zeros(self.n_q, 1);
                command.q_ddot_des = zeros(self.n_q, 1);
            
                for j = 1:self.n_q
                    for coeff_idx = 0:5
                        command.q_des(j) = command.q_des(j) + ...
                            self.alpha(j,coeff_idx+1) * t^coeff_idx;
                        if coeff_idx > 0
                            command.q_dot_des(j) = command.q_dot_des(j) + ...
                                coeff_idx * self.alpha(j,coeff_idx+1) * t^(coeff_idx-1);
                        end
                        if coeff_idx > 1
                            command.q_ddot_des(j) = command.q_ddot_des(j) + ...
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
                command.q_des = self.q_end;
                command.q_dot_des = zeros(self.n_q, 1);
                command.q_ddot_des = zeros(self.n_q, 1);
            end
        end
    end
end