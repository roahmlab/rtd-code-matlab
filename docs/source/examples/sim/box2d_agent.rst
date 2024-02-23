Box2D Agent example
===================

Can be found under scripts/armour/tasks

Usage
-----
*Note: make sure that you add "rtd-code-architecture-base" to path in the workspace and rehash*

To use the BoxAgent, first create the BoxAgentInfo, GenericEntityState, and BoxAgentVisual objects. 
Initialize GenericEntityState with ntates=2 if not so already by default.
Create a new BoxAgent using the created info, state, and visual components.
Feel free to define the movements of the agent by using `boxagent.state.commit_state_data(time, [x y])`

Finally, you can animate the defined movement of the agent using the `animate()` method under BoxAgent. 
By default it will animate at 30fps, 1second per 1t. 
You can also plot at a single moment in time using the `plot()` method under BoxAgent. 

There is a demo file `/scripts/demos/box2d_agent.m` for an example of the BoxAgent.

BoxAgent
--------
Class for creating a simple 2D Box Agent.

**Input:** `BoxAgentInfo, GenericEntityState, BoxAgentVisual`

.. mat:autoclass:: +demos.+box2d.BoxAgent
   :show-inheritance:
   :members:
   :undoc-members:

BoxAgentInfo
------------
Class for storing relevant information related to the BoxAgent

**Input:** `(width), (height), (color)`

.. mat:autoclass:: +demos.+box2d.BoxAgentInfo
   :show-inheritance:
   :members:
   :undoc-members:

BoxAgentVisual
--------------
Class to handle visualization of the BoxAgent. Uses stored info to change looks of the agent.

**Input:** `BoxAgentInfo, GenericEntityState, (options deriving from PatchVisualObject)`

.. mat:autoclass:: +demos.+box2d.BoxAgentVisual
   :show-inheritance:
   :members:
   :undoc-members:


