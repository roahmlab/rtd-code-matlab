Simulation Files and World Container
====================================

In this version of the simulator, we explicitly isolate world as just a container.
It holds what should exist in a single simulation instance and can be exported to/from a file.
At the moment, only the initial configuration of the world can be exported and it relies on the use of the :mat:class:`+rtd.+util.+mixin.Options` mixin.

World Model Container
---------------------
.. mat:autoclass:: +rtd.+sim.+world.WorldModel
   :show-inheritance:
   :members:
   :undoc-members:

Base Simulation
---------------
.. mat:autoclass:: +rtd.+sim.BaseSimulation
   :show-inheritance:
   :members:
   :undoc-members:

Armour Simulation
-----------------
.. mat:autoclass:: +armour.ArmourSimulation
   :show-inheritance:
   :members:
   :undoc-members:

Waitr Simulation
----------------
.. mat:autoclass:: +waitr.WaitrSimulation
   :show-inheritance:
   :members:
   :undoc-members:
