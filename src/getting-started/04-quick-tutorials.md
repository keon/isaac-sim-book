# Quick Tutorials Sprint

Work through key official tutorials to cement fundamentals.

## Recommended Tutorial Sequence

Complete these tutorials from NVIDIA's documentation:

### 1. Hello World (15 min)
[Tutorial Link](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html)

**What you learn:**
- Creating a World
- Adding a cube
- Stepping simulation
- Basic task structure

### 2. Adding a Robot (15 min)
[Tutorial Link](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_adding_a_robot.html)

**What you learn:**
- Loading robot from USD
- Articulation wrapper
- Joint control basics

### 3. Adding a Controller (20 min)
[Tutorial Link](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_adding_a_controller.html)

**What you learn:**
- Franka gripper control
- Pick and place task
- Task state machine

### 4. Sensors Tutorial (20 min)
[Tutorial Link](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/index.html)

**What you learn:**
- Camera setup
- Image capture
- Sensor data access

## Save and Reload a Scene

After building something in the GUI:

**Save:**
1. File → Save As
2. Choose location and name (e.g., `my_scene.usd`)
3. Save

**Reload in Python:**
```python
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage

# Open saved scene
open_stage("/path/to/my_scene.usd")

# Get world and reset
world = World()
await world.reset_async()
```

**Reload in GUI:**
1. File → Open
2. Select your .usd file

## GUI vs Python: When to Use Each

**Use GUI for:**
- Initial scene setup
- Visual debugging
- Asset browsing
- Quick experiments

**Use Python for:**
- Reproducible experiments
- Automated testing
- Training loops
- CI pipelines

**Hybrid workflow:**
1. Build scene in GUI
2. Save as USD
3. Load and control via Python
4. Iterate in Python
5. Return to GUI for debugging

## Standalone vs Extension Mode

**Extension mode** (inside GUI):
- Python runs in Omniverse Kit's event loop
- Use `async/await` for long-running code
- Full GUI available
- Good for development

**Standalone mode** (command line):
- Python controls everything
- Synchronous code works
- Can run headless
- Good for training/CI

```python
# Standalone script structure
from isaacsim import SimulationApp

# Create app (before any other imports)
simulation_app = SimulationApp({"headless": False})

# Now import Isaac modules
from omni.isaac.core import World

# Your code here
world = World()
world.reset()

for i in range(1000):
    world.step(render=True)

# Cleanup
simulation_app.close()
```

## Checkpoint: Part I Complete

At this point you should be able to:

- [ ] Launch Isaac Sim
- [ ] Load a robot from the content browser
- [ ] Move joints manually and during simulation
- [ ] Run Python scripts to control robots
- [ ] Save and reload scenes
- [ ] Choose between GUI and standalone workflows

If any of these are unclear, revisit the relevant chapter before proceeding.

## What's Next

Part II explains USD—the scene format. You need this when:
- Scenes don't load correctly
- Transforms are wrong
- You want to build larger environments

If your simple scenes work fine, you can skip ahead to [Importing Robots](../robot-modeling/08-importing-robots.md) and return to USD when problems arise.

## Chapter Summary

You completed core tutorials, practiced saving/loading scenes, and understand when to use GUI vs Python workflows.

Next: [How Scenes Are Actually Stored](../usd-scene/05-how-scenes-stored.md)—understand USD when scene problems arise.

## References

- [Isaac Sim Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/index.html)
- [Standalone Python](https://docs.omniverse.nvidia.com/isaacsim/latest/manual_standalone_python.html)
- [Extension Development](https://docs.omniverse.nvidia.com/extensions/latest/)
