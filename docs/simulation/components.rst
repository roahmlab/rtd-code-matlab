Components for Entities and Agents
==================================

Components can be categorized as Data Components, Data + Behavior Components, and Behavior
Components. Components should have specific purposes such as holding intrinsic data, or
defining entity dynamics. All behavior components transform entity data for use by either
another component or system and may or may not update entity data at the same time.

Entity-component modeled components are behavior or data-behavior components which should be
explicitly called by the entity update functions. Entity-component-system modeled components
are behavior or data-behavior components which are used by a system which should be explicitly
updated at any point of the simulation loop.

Base Components
---------------
.. mat:automodule:: simulation.agent
   :show-inheritance:
   :members:
   :undoc-members:

Armour Components
-----------------
.. mat:automodule:: implementations.ArmourAgent
   :show-inheritance:
   :members:
   :undoc-members:
   :exclude-members: ArmourAgent

Box Obstacle Components
-----------------------
.. mat:automodule:: implementations.BoxObstacle
   :show-inheritance:
   :members:
   :undoc-members:
   :exclude-members: BoxObstacle