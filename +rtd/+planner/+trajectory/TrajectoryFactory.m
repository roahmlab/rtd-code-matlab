classdef TrajectoryFactory < handle
    methods (Abstract)
        createTrajectory(self, robotState, rsInstances, trajectoryParams, options)
    end
end