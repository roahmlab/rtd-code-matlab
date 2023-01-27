classdef ArmTrajectoryFactory < rtd.planner.trajectory.TrajectoryFactory & handle

    properties
        traj_type {mustBeMember(traj_type,{'piecewise', 'bernstein', 'zerohold'})} = 'piecewise'
        trajOptProps
    end

    methods
        function self = ArmTrajectoryFactory(trajOptProps, traj_type)
            arguments
                trajOptProps
                traj_type {mustBeMember(traj_type,{'piecewise', 'bernstein', 'zerohold'})} = 'piecewise'
            end
            self.trajOptProps = trajOptProps;
            self.traj_type = traj_type;
        end

        % Create a new trajectory object for the given state
        function trajectory = createTrajectory(self, robotState, rsInstances, trajectoryParams, options)
            arguments
                self armour.trajectory.ArmTrajectoryFactory
                robotState rtd.entity.states.ArmRobotState
                rsInstances struct = struct
                trajectoryParams (:,1) double = []
                options.jrsInstance armour.reachsets.JRSInstance = armour.reachsets.JRSInstance.empty()
                options.traj_type {mustBeMember(options.traj_type,{'piecewise', 'bernstein', 'zerohold'})} = self.traj_type
            end
            
            if ~strcmp(options.traj_type, 'zerohold') && isempty(options.jrsInstance)
                try
                    options.jrsInstance = rsInstances.jrs;
                catch
                    error("Must provide handle for some armour.reachsets.JRSInstance if not generating a armour.trajectory.ZeroHoldArmTrajectory!")
                end
            end

            switch options.traj_type
                case 'piecewise'
                    trajectory = armour.trajectory.PiecewiseArmTrajectory(self.trajOptProps, robotState, options.jrsInstance);

                case 'bernstein'
                    trajectory = armour.trajectory.BernsteinArmTrajectory(self.trajOptProps, robotState, options.jrsInstance);

                case 'zerohold'
                    trajectory = armour.trajectory.ZeroHoldArmTrajectory(self.trajOptProps, robotState);
            end

            if ~isempty(trajectoryParams)
                trajectory.setParameters(trajectoryParams)
            end
        end
    end
end