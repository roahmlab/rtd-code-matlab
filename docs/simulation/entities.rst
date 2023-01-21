Entities and Agents
===================

Entities define objects that exist in the simulated world with a variety of properties.
This simulator works on mixed Entity-Component (EC) and Entity-Component-System (ECS)
architecture. Each entity aggregates a few core components with special cases aggregating
any additional necessary components. Entity data purely exists in the components. Components
can be categorized as Data Components, Data + Behavior Components, and Behavior Components.

Because of this, there is no base entity class (as of yet). The following are implemented.

Armour Agent
------------
.. mat:autoclass:: implementations.ArmourAgent.ArmourAgent
   :show-inheritance:
   :members:
   :undoc-members:

Box Obstacle Entity
-------------------
.. mat:autoclass:: implementations.BoxObstacle.BoxObstacle
   :show-inheritance:
   :members:
   :undoc-members:

