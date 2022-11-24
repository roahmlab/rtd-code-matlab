classdef RobotModel < handle
    properties
        robotInfo       % Includes any initialization level information for the robot
    end
    methods
        % This class encapsulates everything needed for a robot that exists
        % in some world to function. Dynamics and other such evolving
        % functions based on the model itself would be better fit here.
        % This will eventually act as an interface to the underlying robot.
        
        % Get an atomic state of the robot at the current time instance.
        % Returns type RobotState
        robotState = getState(self)
        % Set an active trajectory
        % Takes type Trajectory
        setTrajectory(self, trajectory)
    end
end