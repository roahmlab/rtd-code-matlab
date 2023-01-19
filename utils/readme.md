# Utility Classes

- mixins.NamedClass : creates and populates a classname property which is then used in vdisp.
- mixins.UUID : creates and populates a unique identifier for every hard instance of the class.

For example, by including mixins.NamedClass, you can set the verbosity of the message and what verbosity level to display.
The default verbosity level is 0, but you can change verbose_level to -1 to hide all messages.
And by using mixins.UUID, you can ensure every new RobotState generated is actually a unique state, even if they may seem to have the same parameters.
By using this with the handle class, it gives the flexibility of sharing data or generating new data easily.
More specifically, we can use it to ensure that even if the robot seems to output the same state at some other time instance, a caching method won't restore an old object and possibly create undefined behavior.
This is mostly used for the ReachableSets class, where the getReachableSets function will first see if the reachable set was cached already, and if it was, it won't generate a new one.
