# World

In this folder are files that interface with the world for RTD.
At the moment, WorldInfo, WorldModel, and RobotModel are unused, but they're here for future feature reasons.
As such, they'll aslo be discussed below.

---

## WorldModel

This class should be extended as needed to create a model of the world used by RTD.
It should contains a single initialization of WorldInfo, and then will create atomic instances of WorldState as needed as well as evolve world dynamics/predictions.
If other agents start to be more of a concern, it is through the WorldModel that you learn more about these agents.

---

## WorldInfo

This stores common static info about the world.
For example, this might be the bounds on the configuration space for an arm robot.
OR it could be things like the location of a target platform.
If you find any parts of this changing during runtime, it should either be a part of the WorldModel or WorldState!

---

## WorldState

This should hold atomic state instances of the world.
Parts of the world that change over time should be included in this.
For example, obstacles are something that are always an evolving part of the state of the world, so that's going to be in here.
Other possible inclusions would be other agents and their states.

---

## RobotModel

Like the world model, this class encapsulates everything needed for a robot that exists in some world to function.
Dynamics and other such evolving functions based on the model itself would be fit to extend from here.
Eventually, this will act as an interface to the underlying robot in simulation or world.
If it includes dynamics it could also generate state predictions.
An example of something that might go in here is RNEA for a manipulator.
This should include a single instance of RobotInfo, and generate atomic instances of RobotState.

---

## RobotInfo

Parameters and static unchanging aspects of a robot.
Again, if you find any part of this changing during runtime, it should be a part of the RobotModel or RobotState!

---

## RobotState

State of a generic robot at a given moment in time.
Time is always relevant, so it is included in the base object.
For utility, this extends rtd.mixins.UUID, so that every initialization of this class will have a different id, even if they share the same state values.
That also means if you intialize it once, but copy it (so it's a copy of the same state realizaton), it will share the same id.
