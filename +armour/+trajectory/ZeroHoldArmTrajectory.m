classdef ZeroHoldArmTrajectory < rtd.planner.trajectory.Trajectory
    % ZeroHoldArmTrajectory
    % Required properties
    properties (Constant)
        vectorized = true
    end
    % Additional properties
    methods
        % The ZeroHoldArmTrajectory constructor, which simply sets parameters and
        % attempts to call internalUpdate, a helper function made for this
        % class to update all other internal parameters once fully
        % parameterized.
        function self = ZeroHoldArmTrajectory(startState)
            arguments
                startState(1,1) rtd.entity.states.ArmRobotState
            end
            self.startState = startState;
        end
        
        % Set the parameters of the trajectory, with a focus on the
        % parameters as the state should be set from the constructor.
        function setParameters(self, trajectoryParams, options)
            arguments
                self armour.trajectory.ZeroHoldArmTrajectory
                trajectoryParams(1,:) double
                options.startState rtd.entity.states.ArmRobotState = self.startState
            end
            self.trajectoryParams = trajectoryParams;
            self.startState = options.startState;
        end
        
        % Validate that the trajectory is fully characterized
        function valid = validate(self, throwOnError)
            arguments
                self armour.trajectory.ZeroHoldArmTrajectory
                throwOnError(1,1) logical = false
            end
            % Make sure we actually have a robot state to work with.
            valid = ~isempty(self.startState);

            % Throw if wanted
            if ~valid && throwOnError
                errMsg = MException('ZeroHoldArmTrajectory:InvalidTrajectory', ...
                    'Must have some existing robot state to use this!');
                throw(errMsg)
            end
        end
        
        % Computes the actual input commands for the given time.
        % throws RTD:InvalidTrajectory if the trajectory isn't set
        % throws RTD:Trajectory:InvalidTime if the time is before the
        % trajectory exists
        % TODO: write in a vectorized manner
        function command = getCommand(self, time)
            arguments
                self armour.trajectory.ZeroHoldArmTrajectory
                time(1,:) double
            end

            % Do a parameter check and time check, and throw if anything is
            % invalid.
            self.validate(true);
            if any(time < self.startState.time)
                ME = MException('ZeroHoldArmTrajectory:InvalidTime', ...
                    'Invalid time provided to ZeroHoldArmTrajectory');
                throw(ME)
            end
            
            % Make the state
            n_q = length(self.startState.q);
            state = repmat([self.startState.q;0], 1, length(time));
            pos_idx = 1:n_q;
            acc_vel_idx = ones(1,n_q)+n_q;

            % Generate the output.
            command = rtd.entity.states.ArmRobotState(pos_idx, acc_vel_idx, acc_vel_idx);
            command.time = time;
            command.state = state;
        end
    end
end