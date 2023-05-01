# Box2D Agent example
Can be found under scripts/armour/tasks

## Relevant Scripts
> ### BoxAgent
> Class for creating a simple 2D Box Agent.
>
> **Input:** `BoxAgentInfo, GenericEntityState, BoxAgentVisual`
>
> **Methods:**
> - `reset(options)`: - resets components (or override with option)
> - `plot((time), (xlim), (ylim))`: - plot the graph in range x=xlim, y=ylim, at time=t
> - `animate((start_time), (end_time), (speed), (fps))`: - animates from start_time to end_time at 1 second = speed*t

> ### BoxAgentInfo
> Class for storing relevant information related to the BoxAgent
>
> **Input:** `(width), (height), (color)`
>
> **Methods:**
> - `reset(options)`: - resets info (or override with option)

> ### BoxAgentVisual
> Class to handle visualization of the BoxAgent. Uses stored info to change looks of the agent.
> 
> **Input:** `BoxAgentInfo, GenericEntityState, (options deriving from PatchVisualObject)`
>
> **Methods:**
> - `reset(options)`: - resets info (or override with option)
> - `plot((time))`: - plots agent at time=time
> - `plot_at_time(time)`: same as above

## Usage
*Note: make sure that you add "rtd-code-architecture-base" to path in the workspace and rehash*

To use the BoxAgent, first create the BoxAgentInfo, GenericEntityState, and BoxAgentVisual objects. 
Initialize GenericEntityState with ntates=2 if not so already by default.
Create a new BoxAgent using the created info, state, and visual components.
Feel free to define the movements of the agent by using `boxagent.state.commit_state_data(time, [x y])`

Finally, you can animate the defined movement of the agent using the `animate()` method under BoxAgent. 
By default it will animate at 30fps, 1second per 1t. 
You can also plot at a single moment in time using the `plot()` method under BoxAgent. 

There is a demo file `box2d_agent.m` for an example of the BoxAgent.