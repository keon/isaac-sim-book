# Simulation Execution Model

Your control loop runs at the wrong rate. Sensor data is stale. Changes don't take effect. These bugs trace back to Isaac Sim's execution model—how physics, rendering, and callbacks actually run.

This chapter explains what you need to know after hitting these issues.

## The Core Loop

Isaac Sim doesn't run physics and rendering in lockstep. They're decoupled:

```
┌─────────────────────────────────────────────────────────────┐
│                     SIMULATION FRAME                        │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐       │
│  │ Pre-physics │ → │  Physics    │ → │Post-physics │       │
│  │  callbacks  │   │  step(s)    │   │  callbacks  │       │
│  └─────────────┘   └─────────────┘   └─────────────┘       │
│         │                                   │               │
│         ▼                                   ▼               │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐       │
│  │ USD stage   │ → │ Pre-render  │ → │  Rendering  │       │
│  │    sync     │   │  callbacks  │   │             │       │
│  └─────────────┘   └─────────────┘   └─────────────┘       │
└─────────────────────────────────────────────────────────────┘
```

**Key insight**: Multiple physics steps can occur per rendered frame.

If `physics_dt=1/120` and `rendering_dt=1/60`, each `world.step(render=True)` runs **2** physics steps.

## Physics vs Render Rate

```python
from omni.isaac.core import World

# 120 Hz physics, 60 Hz rendering
world = World(physics_dt=1/120, rendering_dt=1/60)
```

This matters because:
- **Physics callbacks** run at 120 Hz (every physics step)
- **Render callbacks** run at ~60 Hz (every render)
- **Camera data** updates after rendering (not after physics)

## Why Your Control Loop is Wrong

**Bug**: Control runs at inconsistent rate.

```python
# WRONG: Render callback for control
def on_render(event):
    error = target - robot.get_position()
    robot.apply_force(error * kp)  # Rate varies with GPU load!
```

On a fast GPU, this runs at 90 Hz. On a slow GPU, 30 Hz. Your control authority changes with frame rate.

**Fix**: Use physics callbacks:

```python
# RIGHT: Physics callback for control
def on_physics_step(step_size):
    error = target - robot.get_position()
    robot.apply_force(error * kp)  # Consistent rate

world.add_physics_callback("controller", on_physics_step)
```

## Why Sensor Data is Stale

**Bug**: Camera data is from the previous frame.

```python
def on_physics_step(dt):
    position = robot.get_world_pose()  # Current
    rgb = camera.get_rgb()  # STALE - from previous render!
```

Cameras render in the render pass, not the physics pass. When you query camera data in a physics callback, you get the previous frame.

**Fix**: Query camera data after rendering:

```python
def on_render(event):
    rgb = camera.get_rgb()  # Fresh after this render
    self.last_rgb = rgb

def on_physics_step(dt):
    # Use cached camera data (accept 1-frame latency)
    if self.last_rgb is not None:
        self.process_image(self.last_rgb)
```

Or structure your pipeline to accept perception latency (which real robots have anyway).

## Why Changes Don't Take Effect

**Bug**: Set a property, read it back, get old value.

```python
robot.set_position([1, 0, 0])
pos = robot.get_position()  # Still [0, 0, 0]!
```

USD operations are queued, not immediate. The change won't appear until the next stage sync.

**Fix**: Force update or accept the delay:

```python
robot.set_position([1, 0, 0])
omni.kit.app.get_app().update()  # Process pending updates
pos = robot.get_position()  # Now [1, 0, 0]
```

## Standalone vs GUI Mode

**Standalone** (Python script):
```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# You control when steps happen
while simulation_app.is_running():
    world.step(render=True)
```

- You control timing
- Steps are synchronous
- Most predictable for control

**GUI/Extension mode**:
- Isaac Sim controls the loop
- Play/Pause buttons control state
- Your code runs via callbacks
- Less predictable timing

For development, use GUI. For training/CI, use standalone.

## Headless Mode

For training workloads, skip the viewport:

```python
simulation_app = SimulationApp({"headless": True})
```

- No viewport rendering
- Camera sensors still work (render offscreen)
- Physics runs at maximum speed

## Step Variants

```python
# Step physics and render
world.step(render=True)

# Step physics only (faster for training)
world.step(render=False)

# Step physics many times, render once
for _ in range(10):
    world.step(render=False)
world.render()
```

For RL training, render=False is 2-5x faster.

## Physics Time vs Wall Time

**Bug**: Using wall-clock time for control.

```python
# WRONG
import time
last_time = time.time()
dt = time.time() - last_time  # Wall time!
```

If simulation runs faster than real-time, physics time advances faster than wall time. If slower, it lags.

**Fix**: Use simulation time:

```python
# RIGHT
current_time = world.current_time
```

## The Warm-Up Problem

First few steps behave differently:
- Physics solver converging on contacts
- Articulations resolving initial constraints
- Sensors initializing

**Fix**: Run warm-up steps before collecting data:

```python
# Warm-up
for _ in range(10):
    world.step()

# Now run real simulation
```

## Callback Registration

```python
# Physics callback (every physics step)
world.add_physics_callback("control", on_physics_step)

# Render callback (every render)
import omni.kit.app
app = omni.kit.app.get_app()
render_sub = app.get_render_event_stream().create_subscription_to_pop(on_render)

# Timeline callback (play/pause/stop events)
timeline_sub = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(on_timeline)
```

## Summary: When to Use Each Callback

| Callback Type | Rate | Use For |
|--------------|------|---------|
| Physics | Fixed (e.g., 120 Hz) | Control, forces, state queries |
| Render | Variable (~30-90 Hz) | Visualization, camera processing |
| Timeline | On events | React to play/pause/stop |

## Chapter Summary

Key execution model facts:
- Physics and rendering are decoupled
- Physics callbacks run at fixed rate (use for control)
- Render callbacks run at variable rate (use for visualization)
- Camera data updates after rendering
- USD changes are queued, not immediate

Next: [Physics Stepping and Reproducibility](./16-physics-stepping-reproducibility.md)—why results differ run to run.

## References

- [Isaac Sim Core API Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html)
- [Omniverse Kit SDK](https://docs.omniverse.nvidia.com/kit/docs/kit-sdk/latest/)
