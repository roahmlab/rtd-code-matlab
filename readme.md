# rtd_planner

This folder contains a proposed structure for the core RTD component of our RTD projects.
This does not include the simulator or the set representation themselves.
Certain specific's, like RobotInfo, WorldState, and WorldInfo are included, but empty in the world subpackage because they're harder to define without specific implementation at the moment.
Each subpackage is like a themed interface:

- trajectories: contains the interfaces for components directly affect by trajectory definition - Trajectory, ReachableSets, and ReachableSetsInstance.
- trajopt: contains components and interfaces for the core optimization problem, including what is being optimized for - Waypoint, Objective, OptimizationEngine, and RTD_TrajOpt.
- world: contains interfaces for how the planner interacts with the rest of world - RobotModel, RobotState, RobotInfo, WorldModel, WorldState, and WorldInfo

Then, within the main package, we have the main RTD_Planner object.

---

An ideal sequence of usage would be:

1. Set up the simulation environment
2. Set up robot & world models based on the simulation environment (They are interfaces)
3. Set up a high level planner
4. Set up the RTD planner & Low level controller
5. For every simulation loop:
    5.1. Execute LLC and step the simulation
    5.2. If time:
        5.2.1. Get the next waypoint from the HLP if time
        5.2.2. Plan a new trajectory if new waypoint
    5.3. Update state variables / Vis

---

Intended usage to extend this to any project is to create specialized versions of each of the following classes:

- Trajectory : the parameterized trajectory
- ReachableSets (+ReachableSetsInstance) : reachable set generation/selection + obstacle intersection
- Waypoint : the goal position
- Objective : the cost function
- RobotModel (+RobotState, RobotInfo) : interface to the robot
- WorldModel (+WorldState, WorldInfo) : interface to the world
- RTD_Planner : the planner itself that would be called each t_plan

Down the line, Objective will likely be split to the raw cost itself and the underlying dynamics.
In that situation, the cost could become a component of the waypoint class, and the dynamics could be its own thing, or a part of the trajectory class.

