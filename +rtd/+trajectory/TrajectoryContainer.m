classdef TrajectoryContainer < handle
    properties (SetAccess=private)
        startTimes(1,:) double = [inf]
        trajectories(1,:) cell = {}
    end
    methods
        function setInitialTrajectory(self, initialTrajectory)
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
            if self.isValid()
                self.startTimes = [0, inf];
                self.trajectories = self.trajectories(1);
            else
                warning('clear() for rtd.trajectory.TrajectoryContainer called before valid initial trajectory was set!')
            end
        end
        function valid = isValid(self, throwIfInvalid)
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
        function setTrajectory(self, trajectory)
            arguments
                self(1,1) rtd.trajectory.TrajectoryContainer
                trajectory(1,1) rtd.trajectory.Trajectory
            end
            self.isValid(true);
            % Add the trajectory if it is valid
            if trajectory.validate()
                self.startTimes(end) = trajectory.startTime;
                self.startTimes = [self.startTimes, inf];
                self.trajectories = [self.trajectories, {trajectory}];
            else
                warning('Invalid trajectory provided to rtd.trajectory.TrajectoryContainer')
            end
        end
        function output = getCommand(self, time)
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