Simulation Systems
==================

Components that are more tailored to the ECS architecture adapt data for
simulation systems that run during the simulation loop, separate of each
entity. These systems are all expected to track time as well to ensure
the simulation retains time synchronization while enabling flexibility
to call systems whenever relevant.

Patch3dCollisionSystem
----------------------
.. mat:automodule:: +rtd.+sim.+systems.+patch3d_collision
   :show-inheritance:
   :members:
   :undoc-members:

PatchVisualSystem
-----------------
.. mat:automodule:: +rtd.+sim.+systems.+patch_visual
   :show-inheritance:
   :members:
   :undoc-members:

RandomArmConfigurationGoal
--------------------------
.. mat:autoclass:: implementations.ArmGoalSystem.RandomArmConfigurationGoal
   :show-inheritance:
   :members:
   :undoc-members:
