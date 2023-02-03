# rtd.planner

This folder contains a proposed structure for the core RTD component of our RTD projects.
This does not include the simulator or the set representation themselves.
Certain specific's, like RobotInfo, WorldState, and WorldInfo are included, but empty in the world subpackage because they're harder to define without specific implementation at the moment.
Each subpackage is like a themed interface:

- reachablesets: contains the interfaces for each of the reachable set generators and the instances which then generate their nonlinear constraints - ReachableSets and ReachableSetsInstance.
- trajectories: contains the interfaces for components directly affect by trajectory definition - Trajectory.
- trajopt: contains components and interfaces for the core optimization problem, including what is being optimized for - Waypoint, Objective, OptimizationEngine, RTD_TrajOpt, and TrajOptProps.
- utils: contains some utility classes that can also be subclassed for some quick useful functionality - rtd.util.mixins.NamedClass and rtd.util.mixins.UUID.
- world: contains interfaces for how the planner interacts with the rest of world - RobotModel, RobotState, RobotInfo, WorldModel, WorldState, and WorldInfo

Within the main package, we have the main RTD_Planner object.

Then, in the implementations folder is the example implementation of Armour.

---

An ideal sequence of usage would be:

1. Set up the simulation environment
2. Set up robot & world models based on the simulation environment (They are interfaces)
3. Set up a high level planner
4. Decide on a the general trajopt parameters and update TrajOptProps
5. Set up the RTD planner & Low level controller
6. For every simulation loop:
    6.1. Execute LLC and step the simulation
    6.2. If time:
        5.2.1. Get the next waypoint from the HLP if time
        5.2.2. Plan a new trajectory if new waypoint
    6.3. Update state variables / Vis

---

Intended usage to extend this to any project is to create specialized versions of each of the following classes:

- Trajectory : the parameterized trajectory
- ReachableSets (+ReachableSetsInstance) : reachable set generation/selection + obstacle intersection
- Waypoint : the goal position
- Objective : the cost function
- RobotModel (+RobotState, RobotInfo) : interface to the robot
- WorldModel (+WorldState, WorldInfo) : interface to the world
- RTD_Planner : the planner itself that would be called each t_plan

Important to note is that RTD_TrajOpt isn't include in here, because it's effectively the core logic.
That takes in all the reachable sets and combines their nonlinear constraints to create *one* trajectory optimization problem which it solves and returns to the RTD_Planner class.
This enabled flexibility for projects like the rover team's where they may require the optimization of multiple trajectory types at once.
The selection criteria between those trajectories would then be done within the class that extends RTD_Planner.

Also important to note is that down the line, Objective will likely be split to the raw cost itself and the underlying dynamics.
In that situation, the cost could become a component of the waypoint class, and the dynamics could be its own thing, or a part of the trajectory class.

