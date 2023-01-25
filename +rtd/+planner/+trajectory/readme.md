# Trajectories

Inside of the trajectories abstract class, you'll find 3 abstract functions and 1 concrete function.

Abstract:
- valid: bool = validate(throwOnError: bool)
- setTrajectory(trajectoryParams: vector, rsInstances: cells, robotState: RobotState, varargin)
- command: vector =  getCommand(time: double)

Concrete:
- [TrajectoryParams: vector, startState: struct] = getTrajParams()

Within each specialization, a helper function for internal updates is reasonable to reduce code duplication.

This class generalizes the trajectory generation so that given any parameter and setup create a starting state, we can create a command value for any given point of time since the start state.
This way, each trajectory encapsulates purely the trajectory generation and resulting command.
Additional specific documentation can be found within the Trajectory.m file.