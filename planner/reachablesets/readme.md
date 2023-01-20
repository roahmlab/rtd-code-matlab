# Reachable Sets

In this folder are the base classes for the reachable set generators and their instance classes.
They're isolated because each instance may then be further modified to create the nonlinear constraints, and each generator may be used across multiple trajopt solvers.
Furthermore, ReachableSets includes a concrete function, getReachableSets, which should be used for getting the reachable set for a robot state instead of bypassing it to call the abstract generateReachableSets function.
getReachableSets handles automatic internal caching of generated reachable sets, so that if wanted, the reachable sets will not be regenerated if the exact same objects are passed into it another time.
This relies on the classes used having a uuid property, which can easily be added by adding rtd.mixins.UUID an additional base class to any state related class used for reachable set generation.

In order to increase the cache size used for getReachableSets, the derived class should increase the value of cache_max_size.
To disable the cache, that value should be 0.
It is disabled by default.

**Note:** If a generated nonlinear constraint function is not atomic, in that it may change class properties, the cache for the generating RS class should be disabled by setting cache_max_size to 0.