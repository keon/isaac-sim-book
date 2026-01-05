# First Python Control

Run a Python script to control the robot programmatically.

## The World Object

Isaac Sim's Python API centers on the `World` object:

```python
from omni.isaac.core import World

# Create or get existing world
world = World()

# Reset initializes physics and resets to initial state
await world.reset_async()

# Step advances simulation by one physics step
world.step(render=True)
```

The World manages:
- Physics scene
- Simulation time
- Registered objects (robots, sensors)
- Callbacks

## Minimal Control Script

Create a new Python file or use Script Editor:

```python
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import numpy as np

async def main():
    # Get world instance
    world = World.instance()
    await world.reset_async()
    
    # Get robot articulation
    franka = world.scene.get_object("Franka")
    if franka is None:
        franka = Articulation(prim_path="/World/Franka", name="Franka")
        world.scene.add(franka)
        await world.reset_async()
    
    # Control loop
    for i in range(500):
        # Set joint targets (7 arm joints + 2 gripper)
        targets = np.array([0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8, 0.04, 0.04])
        franka.set_joint_position_targets(targets)
        
        # Step simulation
        world.step(render=True)

# Run in Isaac Sim's async context
import asyncio
asyncio.ensure_future(main())
```

## Running Scripts

**From Script Editor:**
1. Window → Script Editor
2. Paste code
3. Click Run

**From terminal (standalone):**
```bash
# Using Isaac Sim's Python
~/.local/share/ov/pkg/isaac-sim-4.5.0/python.sh my_script.py
```

**As extension:**
Scripts can be packaged as Omniverse extensions for integration with the GUI.

## Stepping the World

Key stepping methods:

```python
# Single step with rendering
world.step(render=True)

# Single step without rendering (faster)
world.step(render=False)

# Multiple steps
for _ in range(100):
    world.step(render=False)
world.render()  # Render once at end
```

Rendering is expensive. Skip it during fast rollouts.

## Reading Robot State

```python
# Joint positions (radians for revolute, meters for prismatic)
positions = franka.get_joint_positions()

# Joint velocities
velocities = franka.get_joint_velocities()

# End effector pose (if defined)
ee_pos, ee_rot = franka.get_world_pose()

# Applied joint efforts (torques/forces)
efforts = franka.get_applied_joint_efforts()
```

## Setting Control Targets

```python
# Position control (uses drive stiffness/damping)
franka.set_joint_position_targets(target_positions)

# Velocity control
franka.set_joint_velocity_targets(target_velocities)

# Direct effort (torque) control
franka.set_joint_efforts(efforts)
```

Position targets work through the joint drives—physics interpolates to the target. Direct efforts bypass drives for torque-level control.

## A Moving Robot

Make the robot wave:

```python
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

async def wave():
    world = World.instance()
    await world.reset_async()
    
    franka = Articulation("/World/Franka")
    franka.initialize()
    
    home = np.array([0, -0.5, 0, -2.0, 0, 1.5, 0.8, 0.04, 0.04])
    
    for t in range(1000):
        # Oscillate joint 4
        wave_offset = 0.5 * np.sin(t * 0.05)
        target = home.copy()
        target[3] += wave_offset
        
        franka.set_joint_position_targets(target)
        world.step(render=True)

import asyncio
asyncio.ensure_future(wave())
```

## Common First Errors

**"Articulation not initialized"**: Call `franka.initialize()` or `world.reset_async()` first.

**Robot doesn't move**: Check that simulation is playing. Check joint limits—target might be outside range.

**Robot explodes**: Gains too high, timestep too large, or collision issues. Covered in [Collision and Contact Tuning](../robot-modeling/10-collision-contact.md).

**Script hangs**: In GUI mode, use `async/await` pattern. In standalone, use synchronous `world.reset()`.

## Chapter Summary

You controlled a robot through Python: creating a World, stepping simulation, reading state, and setting joint targets. You understand the difference between position targets (through drives) and direct efforts.

Next: [Quick Tutorials Sprint](./04-quick-tutorials.md)—run through official tutorials to solidify basics.

## References

- [Core API Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html)
- [Articulation API](https://docs.omniverse.nvidia.com/py/isaacsim/source/isaacsim.core/isaacsim.core.articulations.html)
- [Standalone Python](https://docs.omniverse.nvidia.com/isaacsim/latest/manual_standalone_python.html)
