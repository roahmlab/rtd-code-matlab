classdef ArmTrajectoryFactory < rtd.trajectory.TrajectoryFactory & handle
% Factory for creating trajectories for the arm as used in ARMOUR
%
% --- More Info ---
% Author: Adam Li (adamli@umich.edu)
% Written: 2023-01-27
% Last Updated: 2023-10-04 (Adam Li)
%
% See also: rtd.trajectory.mTrajectoryFactory, armour.trajectory.PiecewiseArmTrajectory,
% armour.trajectory.BernsteinArmTrajectory, armour.trajectory.ZeroHoldArmTrajectory
% armour.trajectory.TwoBernsteinArmTrajectory
%
% --- Revision History ---
% 2023-10-04 - Added support for two bernstein parameterization trajectories.
% 2023-09-07 - removed dependency on armour.reachsets.JRSInstance for each trajectory
%
% --- More Info ---
%

    properties
        % the default trajectory type to use
        traj_type {mustBeMember(traj_type,{'piecewise', 'bernstein', 'twobernstein', 'zerohold'})} = 'piecewise'
    end

    methods
        function self = ArmTrajectoryFactory(trajOptProps, traj_type)
            % Constructor for the ArmTrajectoryFactory
            %
            % Arguments:
            %   trajOptProps (rtd.planner.trajopt.TrajOptProps): Trajectory optimization properties
            %   traj_type (str): The type of trajectory to use. Must be one of
            %       'piecewise', 'bernstein', 'twobernstein', or 'zerohold'
            %
            arguments
                trajOptProps rtd.planner.trajopt.TrajOptProps
                traj_type {mustBeMember(traj_type,{'piecewise', 'bernstein', 'twobernstein', 'zerohold'})} = 'piecewise'
            end

            self.trajOptProps = trajOptProps;
            self.traj_type = traj_type;
        end

        % Create a new trajectory object for the given state
        function trajectory = createTrajectory(self, robotState, rsInstances, trajectoryParams, options)
            % Create a new trajectory object for the given state
            %
            % Arguments:
            %   robotState (rtd.entity.states.ArmRobotStateInstance): The robot state
            %   rsInstances (struct, optional): The reach set instances
            %   trajectoryParams (double, optional): The trajectory parameters
            %   options: Keyword arguments. See Below.
            %
            % Keyword Arguments:
            %   jrsInstance (armour.reachsets.JRSInstance): The joint reach set instance
            %   traj_type (str): The type of trajectory to use. Must be one of
            %       'piecewise', 'bernstein', 'twobernstein', or 'zerohold'. If not specified, the
            %       default trajectory type is used.
            %
            % Returns:
            %   trajectory (rtd.trajectory.Trajectory): The trajectory object
            %
            arguments
                self armour.trajectory.ArmTrajectoryFactory
                robotState rtd.entity.states.ArmRobotStateInstance
                rsInstances struct = struct
                trajectoryParams (:,1) double = []
                options.jrsInstance armour.reachsets.JRS.JRSInstance = armour.reachsets.JRS.JRSInstance.empty()
                options.traj_type {mustBeMember(options.traj_type,{'piecewise', 'bernstein', 'twobernstein', 'zerohold'})} = self.traj_type
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

                case 'twobernstein'
                    trajectory = armour.trajectory.TwoBernsteinArmTrajectory(robotState, ...
                        self.trajOptProps.planTime, ...
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