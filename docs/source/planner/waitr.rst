Example RTD Planner Implementation of WAITR
===========================================

WaitrPlanner and these related components define the WAITR planner as described by Brei et. al. in https://arxiv.org/abs/2309.03111.
This planner extends ArmourPlanner and requires the use of the matching robust controller.
It still uses the same sets of trajectories that ARMOUR has.

The Base WAITR Planner
----------------------
.. mat:autoclass:: +waitr.WaitrPlanner
   :show-inheritance:
   :members:
   :undoc-members:

Extra Reachable Sets not in ARMOUR
----------------------------------
.. mat:automodule:: +waitr.+reachsets
   :show-inheritance:
   :members:
   :undoc-members:
