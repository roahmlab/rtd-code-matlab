# Trajopt classes

This folder contains the classes core to the trajopt problem.
At the moment, Waypoint is unused, but remains because it may be a useful abstraction.
Each of the names are fairly self explanatory:

- Objective
- OptimizationEngine
- RTD_TrajOpt
- TrajOptProps
- Waypoint

Since the trajopt problem is core to RTD, these classes actually should need to least extra work or extension.

---

Objective abstracts away the objective function used for the optimization.
Any parameters that might be useful for the generation of an objective function call should be included in a class that extends it, and then it should take care to ensure every objective callback generated is **atomic**.
Bein atomic means that that callback should make zero modifications to the Objective function generating object.
This is to ensure there are no issues in case parallelization happens down the line.

OptimizationEngine abstracts away the requirements of whatever optimization engine we want to use.
It's modeled off of fmincon, and an example of the fmincon version can be found in the Armour implementation.
This way, the remaining files can remain almost entirely untouched, without any need for extension either.

---

## RTD_TrajOpt

In here, the core trajopt problem for a singular trajectory is setup and solved.
It will automatically combine all generated nonlinear constraints and handle initial parameter sizing.
Ideally, this optimization could then be run on another thread while either other optmizations take place, or the simulation runs itself.

## TrajOptProps

These are the consistent properties we use across RTD for trajectory optimization, and they find themselves used all across the codebase.
This should also rarely change.
Inside are:

- timeForCost: The time used for evaluation in the cost function.
- planTime: The time duration of the nominal plan. Braking action should occur in horizonTime-planTime.
- horizonTime: The time of the overall trajectory until stop.
- doTimeout: Whether or not the timeout the optimization if it is taking too long.
- timeoutTime: The time at which the optimization should timeout.
- randomInit: Whether or not unknown or extra parameters (slack variables or if no initial guess is given) should be randomized or zero initialized.
