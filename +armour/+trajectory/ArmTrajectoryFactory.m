classdef ArmTrajectoryFactory < rtd.trajectory.TrajectoryFactory & handle

    properties
        traj_type {mustBeMember(traj_type,{'piecewise', 'bernstein', 'zerohold'})} = 'piecewise'
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
                    trajectory = armour.trajectory.PiecewiseArmTrajectory(robotState, ...
                        self.trajOptProps.planTime, ...
                        self.trajOptProps.horizonTime, ...
                        options.jrsInstance.num_parameters);
                    paramScale = rtd.util.RangeScaler(options.jrsInstance.input_range, options.jrsInstance.output_range);
                    trajectory.setParamScale(paramScale)

                case 'bernstein'
                    trajectory = armour.trajectory.BernsteinArmTrajectory(robotState, ...
                        self.trajOptProps.horizonTime, ...
                        options.jrsInstance.num_parameters);
                    paramScale = rtd.util.RangeScaler(options.jrsInstance.input_range, options.jrsInstance.output_range);
                    trajectory.setParamScale(paramScale)

                case 'zerohold'
                    trajectory = armour.trajectory.ZeroHoldArmTrajectory(robotState);
            end

            if ~isempty(trajectoryParams)
                trajectory.setParameters(trajectoryParams)
            end
        end
    end
end