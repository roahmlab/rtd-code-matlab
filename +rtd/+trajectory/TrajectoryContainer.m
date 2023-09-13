classdef TrajectoryContainer < handle
% Container class for holding a set of trajectories and consolidating their commands
%
% TrajectoryContainer is a class that holds a set of trajectories and
% provides a method to generate a command based on the time. This class is
% ensure that the commands are generated based on the correct trajectory.
%
% Note:
%   `setInitialTrajectory` must be called before any other methods are
%   called. The return type of the initial trajectory must be the same as
%   the return type of the other trajectories.
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-09-12
%
% See also: rtd.trajectory.Trajectory
%
% --- More Info ---
%

    properties (SetAccess=private)
        % startTimes is a vector of start times for each trajectory
        % The last element is always inf to ensure that the last trajectory is always used
        startTimes(1,:) double = [inf]

        % trajectories is a cell array of trajectories that correspond to the start times
        trajectories(1,:) cell = {}
    end

    methods
        function setInitialTrajectory(self, initialTrajectory)
            % sets the initial trajectory for the container
            %
            % setInitialTrajectory sets the initial trajectory for the container.
            % This method must be called before any other methods are called.
            %
            % Arguments:
            %   initialTrajectory (rtd.trajectory.Trajectory): The initial trajectory
            %
            % Raises:
            %   BadInitialTrajectory: If the initial trajectory is invalid or doesn't start at 0
            %
            arguments
                self(1,1) rtd.trajectory.TrajectoryContainer
                initialTrajectory(1,1) rtd.trajectory.Trajectory
            end

            if initialTrajectory.startTime ~= 0
                errMsg = MException('TrajectoryContainer:BadInitialTrajectory', ...
                    'Provided initial trajectory does not start at 0!');
                throw(errMsg)
            elseif ~initialTrajectory.validate()
                errMsg = MException('TrajectoryContainer:BadInitialTrajectory', ...
                    'Provided initial trajectory is invalid!');
                throw(errMsg)
            end
            if ~self.isValid()
                self.startTimes = [initialTrajectory.startTime, inf];
                self.trajectories = {initialTrajectory};
            else
                self.startTimes(1) = initialTrajectory.startTime;
                self.trajectories{1} = initialTrajectory;
            end
        end

        function clear(self)
            % clears the container
            %
            % `clear` clears the container and resets it to the initial state.
            % It's expected that there is already some initial trajectory set.
            % If not, a warning is shown.
            %
            if self.isValid()
                self.startTimes = [0, inf];
                self.trajectories = self.trajectories(1);
            else
                warning('clear() for rtd.trajectory.TrajectoryContainer called before valid initial trajectory was set!')
            end
        end

        function valid = isValid(self, throwIfInvalid)
            % Checks if the container is valid
            %
            % `isValid` checks if the container is valid. If `throwIfInvalid` is
            % true, then an error is thrown if the container is invalid. The container
            % is considered invalid if the initial trajectory hasn't been set.
            %
            % Arguments:
            %   throwIfInvalid (logical, optional): If true, an error is thrown if the container is invalid
            %
            % Returns:
            %   valid (logical): True if the container is valid, false otherwise
            %
            % Raises:
            %   InvalidContainer: If the container is invalid and `throwIfInvalid` is true
            %
            arguments
                self(1,1) rtd.trajectory.TrajectoryContainer
                throwIfInvalid(1,1) logical = false
            end

            valid = length(self.startTimes) >= 2 && length(self.trajectories) >= 1;
            if ~valid && throwIfInvalid
                errMsg = MException('TrajectoryContainer:InvalidContainer', ...
                    'Initial trajectory for the container has not been set!');
                throw(errMsg)
            end
        end

        function setTrajectory(self, trajectory, throwIfInvalid)
            % Sets a new trajectory for the container to the end
            %
            % `setTrajectory` sets a new trajectory for the container to the end.
            % The new trajectory must start at the same time or after the end of the
            % last trajectory. If the new trajectory is invalid, a warning is shown.
            %
            % Arguments:
            %   trajectory (rtd.trajectory.Trajectory): The new trajectory to add
            %   throwIfInvalid (logical, optional): If true, an error is thrown if the trajectory is invalid
            %
            % Raises:
            %   InvalidTrajectory: If the new trajectory is invalid or starts before the end of the last trajectory and `throwIfInvalid` is true
            %
            arguments
                self(1,1) rtd.trajectory.TrajectoryContainer
                trajectory(1,1) rtd.trajectory.Trajectory
                throwIfInvalid(1,1) logical = false
            end

            self.isValid(true);
            % Add the trajectory if it is valid
            if trajectory.validate() && trajectory.startTime >= self.startTimes(end-1)
                self.startTimes(end) = trajectory.startTime;
                self.startTimes = [self.startTimes, inf];
                self.trajectories = [self.trajectories, {trajectory}];
            elseif throwIfInvalid
                errMsg = MException('TrajectoryContainer:InvalidTrajectory', ...
                    'Provided trajectory starts before the end of the last trajectory!');
                throw(errMsg)
            else
                warning('Invalid trajectory provided to rtd.trajectory.TrajectoryContainer')
            end
        end

        function output = getCommand(self, time)
            % Generates a command based on the time
            %
            % `getCommand` generates a command based on the time. The command is
            % generated based on the trajectory that is active at the time. If the
            % time is before the start of the first trajectory, then the command is
            % generated based on the initial trajectory. If the time is after the end
            % of the last trajectory, then the command is generated based on the last
            % trajectory.
            %
            % Arguments:
            %   time (double): The times to generate the commands for
            %
            % Returns:
            %   output: The generated commands
            %
            arguments
                self(1,1) rtd.trajectory.TrajectoryContainer
                time(1,:) double
            end
            
            self.isValid(true);
            % Generate an output trajectory based on the provided
            output(length(time)) = self.trajectories{1}.getCommand(0);
            idx_vec = 1:length(time);
            for i=1:(length(self.startTimes)-1)
                mask = (time >= self.startTimes(i) & time < self.startTimes(i+1));
                if sum(mask) == 0
                    continue
                elseif self.trajectories{i}.vectorized
                    output(mask) = self.trajectories{i}.getCommand(time(mask));
                else
                    % Iterate through the relevant indices
                    for j=idx_vec(mask)
                        output(j) = self.trajectories{i}.getCommand(time(j));
                    end
                end
            end
        end
    end
end