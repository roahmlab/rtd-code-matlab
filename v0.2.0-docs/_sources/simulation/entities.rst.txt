Entities and Agents
===================

Entities define objects that exist in the simulated world with a variety of properties.
This simulator works on mixed Entity-Component (EC) and Entity-Component-System (ECS)
architecture. Each entity aggregates a few core components with special cases aggregating
any additional necessary components. Entity data purely exists in the components. Components
can be categorized as Data Components, Data + Behavior Components, and Behavior Components.

To see a demo for making your own basic entity, refer to the Box2D Agent example in the :doc:`/examples/sim/box2d_agent`.

Base Entity Class (WorldEntity)
-------------------------------
All entities derive from the WorldEntity, which prescribes the requirement for some info component and state component.
WorldEntity also provides a handful of utility functions to assist with the composition of each entity.
Additional components can be added as desired, and this is demonstrated in the following concrete entities.

.. mat:autoclass:: +rtd.+sim.+world.WorldEntity
   :show-inheritance:
   :members:
   :undoc-members:

Box Obstacle Entity
-------------------
.. mat:autoclass:: +rtd.+entity.BoxObstacle
   :show-inheritance:
   :members:
   :undoc-members:

Armour Agent
------------
.. mat:autoclass:: +armour.ArmourAgent
   :show-inheritance:
   :members:
   :undoc-members:

Waitr Agent
------------
.. mat:autoclass:: +waitr.WaitrAgent
   :show-inheritance:
   :members:
   :undoc-members:
