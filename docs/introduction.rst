Introduction
============

The current version of the rewritten codebase you are looking at is v0.0.1. It is subject to
significant changes in the near futures which may need code rewrites. This MATLAB implementation
supports r2021b and later. 

In here, the code is split up into many different, with a flattened out structure. Changing out
smaller behaviors of an agent or simulator now involves looking at files that pertain to that
specific behavior instead of the whole simulator or agent.

Additionally, the code is written with the intention of making it more explicit. For example,
in the agent's `update` function, you now write out all components involved in the update of the
agent's internal behavior. In the simulator, you write out the update calls to entities and systems
manually, and can update multiple times per timestep.

Time should be tracked across everything to (1) make it easier to manage time, and (2) enable
validation functions to make sure all involved parts are updated accordingly.
