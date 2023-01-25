 Armour planner implementation example

In this folder is an example reimplementation of Armour into this template.
The extras folder then demonstrates how you might go ahead and wrap this to work with the old RTD simulator code.
Specifically, armour.planner_wrapper will show how it can be wrapped fairly easily.

To run this code, you'll need to add to your path:

- CORA
- dynamics-dev
- armtd-dev
- polynomial_zonotope_ROAHM

Then, run fetch_random_example_new in the extras folder to see just the wrapped planner in use, or fetch_validate_new_planner to see that they two generate the same trajectories.
If running the validation code, add a breakpoint to line 299 of uarmtd_planner_wrapped_comparison.m in extras/.

---

Within the reachablesets folder, the ReachableSets subclassed classes are

- JointReachableSetsOnline: wraps the create_jrs_online function.
- ForwardOccupancy: requires a handle to an instance of JointReachableSetsOnline, where it'll get the relevant JRS and build the forward occupany from it
- InputReachableSet: requires a handle to an instance of JointReachableSetsOnline, which it sends to RNEA and generates the ub and lb for the input. while this isn't exactly correct since it should be storing the generated reachable set to an IRSInstance, it demonstrates the flexibility.

Each of these generally take in some robot state and other info to build out reachable sets which are then stored in their respective instances.
Then those instances take the world state where it identifies how to slice the reachable sets and create the nonlinear constraints.
The resulting reachable set instances also include their expected parameter sizes.
Although not implemented yet as it's a new feature, this allows the RTD_TrajOpt class to validate that all reachable sets are "compatible" and to automatically resize the parameters for any slack variables if wanted.

Also note that the reachablesets do not extend the getReachableSet function, but instead extend the generateReachableSet function.
getReachableSets is supposed to be the primary function you call to get the reachable sets, and it handles the caching and restoration of prior reachable sets depending on the cache_max_size.
This way, the reachablesets class encapsulates the generation and recollection of reachable sets, and the reachablesetinstances classes all handle the instance specific nonlinear constraint generation and utility function.
This is most obvious with the FOInstance class where there are multiple helper functions in the class for the constrains.

Within this framework, there is still room for flexibility in implementation.
JointReachableSetsOnline and JRSInstance shows one way of doing the generation-instance split, while ForwardOccupancy and FOInstance shows another way.

---

Within the trajectories folder, the original armtd and new bernstein trajectories are implemented as ArmTdTrajectory and ArmourBernsteinTrajectory respectively.
These show roughly how the trajectories should be written, with ArmTdTrajectory currently showing the most complete and ideal implementation.

---

Within the trajopt folder, there is a GenericArmObjective, which sets an objective to the norm of the joint space from any time position used for cost, and an FminconOptimizationEngine object.
FminconOptimizationEngine has within it a function to terminate optimization early if wanted, and is a demo of how OptimizationEngine should be used to wrape other optimizers.

---

World only has ArmRobotInfo and ArmRobotState for now, as that is all that matters to the planner written so far.
These save static unchanging data and properties that evolve over time, respectively.