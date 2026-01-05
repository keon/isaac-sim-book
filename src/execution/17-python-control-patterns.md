# Python Control Patterns

Resets don't fully reset. State accumulates across episodes. Callbacks execute in wrong order.

This chapter covers patterns that avoid these issues.

## The World and Reset

```python
from omni.isaac.core import World

world = World()

# Initial reset - initializes everything
await world.reset_async()

# Step the simulation
for _ in range(1000):
    world.step(render=True)

# Reset for next episode
await world.reset_async()
```

`reset_async()` does:
- Resets simulation time to 0
- Resets physics state to initial configuration
- Re-initializes registered objects
- Calls registered reset callbacks

## Reset Pitfall: Incomplete Reset

**Bug**: Robot doesn't return to initial pose.

```python
# Initial pose
robot.set_joint_positions([0, 0, 0, 0, 0, 0])
await world.reset_async()

# Run episode
for _ in range(500):
    robot.set_joint_position_targets(random_targets)
    world.step()

# Reset
await world.reset_async()
# Robot may NOT be at [0,0,0,0,0,0]!
```

`reset_async()` resets to the state at the last `reset_async()`, not necessarily your intended initial state.

**Fix**: Explicitly set state after reset:

```python
await world.reset_async()
robot.set_joint_positions(initial_positions)
robot.set_joint_velocities(np.zeros(6))
```

## State Management

Track what state you're managing:

```python
class EpisodeManager:
    def __init__(self, world, robot):
        self.world = world
        self.robot = robot
        self.initial_positions = np.zeros(6)
        self.step_count = 0
    
    def reset(self):
        """Full reset to known state."""
        # Reset physics
        self.world.reset()
        
        # Reset robot state
        self.robot.set_joint_positions(self.initial_positions)
        self.robot.set_joint_velocities(np.zeros(6))
        
        # Reset your state
        self.step_count = 0
    
    def step(self, action):
        self.robot.set_joint_position_targets(action)
        self.world.step(render=True)
        self.step_count += 1
        return self.get_observation()
```

## Task Pattern

Isaac Sim's Task class provides structure:

```python
from omni.isaac.core.tasks import BaseTask

class PickTask(BaseTask):
    def __init__(self, name):
        super().__init__(name=name)
        self.robot = None
        self.target_object = None
    
    def set_up_scene(self, scene):
        """Called once when scene is set up."""
        # Add robot
        self.robot = scene.add(Articulation(
            prim_path="/World/Robot",
            name="robot"
        ))
        # Add target object
        self.target_object = scene.add(RigidPrim(
            prim_path="/World/Target",
            name="target"
        ))
    
    def pre_step(self, time_step_index, simulation_time):
        """Called before each physics step."""
        pass
    
    def post_reset(self):
        """Called after world.reset()."""
        # Reset task-specific state
        self.target_object.set_world_pose(
            position=np.array([0.5, 0, 0.1])
        )
    
    def get_observations(self):
        """Return current observation."""
        return {
            "robot_pos": self.robot.get_joint_positions(),
            "target_pos": self.target_object.get_world_pose()[0]
        }
```

Usage:
```python
world = World()
task = PickTask("pick_task")
world.add_task(task)
await world.reset_async()  # Calls task.post_reset()

while True:
    obs = task.get_observations()
    # Compute action from obs
    world.step()  # Calls task.pre_step()
```

## Callback Ordering

Multiple callbacks can be registered:

```python
world.add_physics_callback("controller", controller_callback)
world.add_physics_callback("logger", logger_callback)
```

**Ordering**: Callbacks execute in registration order. If `logger` needs data from `controller`, register `controller` first.

**Cleanup**: Remove callbacks when done:

```python
world.remove_physics_callback("controller")
```

## Async vs Sync

In GUI mode, use async:

```python
async def run_episode():
    await world.reset_async()
    for _ in range(100):
        world.step()
    return get_result()

# Run from extension
asyncio.ensure_future(run_episode())
```

In standalone mode, sync works:

```python
world.reset()  # Synchronous
for _ in range(100):
    world.step()
```

## Parallel Environments

For training, run multiple environments:

```python
from omni.isaac.lab.envs import ManagerBasedRLEnv

class MyEnv(ManagerBasedRLEnv):
    def __init__(self, cfg, num_envs=4096):
        super().__init__(cfg)
        self.num_envs = num_envs
    
    def step(self, actions):
        # Actions: [num_envs, action_dim]
        self._apply_actions(actions)
        self.sim.step()
        obs = self._get_observations()
        rewards = self._compute_rewards()
        dones = self._check_terminations()
        return obs, rewards, dones, {}
```

Each environment is a clone with independent state. The physics engine handles them in parallel.

## Common Pattern: Control at Physics Rate

```python
class RobotController:
    def __init__(self, world, robot):
        self.robot = robot
        self.target = None
        
        # Register at physics rate
        world.add_physics_callback("control", self._on_physics)
    
    def set_target(self, position):
        """Called from your code (any rate)."""
        self.target = position
    
    def _on_physics(self, dt):
        """Runs every physics step (fixed rate)."""
        if self.target is None:
            return
        
        current = self.robot.get_joint_positions()
        error = self.target - current
        torque = 100 * error  # Simple P control
        
        self.robot.apply_joint_efforts(torque)
```

## Common Pattern: Perception at Render Rate

```python
class PerceptionSystem:
    def __init__(self, camera):
        self.camera = camera
        self.last_detection = None
        
        # Register at render rate
        app = omni.kit.app.get_app()
        self._sub = app.get_render_event_stream().create_subscription_to_pop(
            self._on_render
        )
    
    def _on_render(self, event):
        """Runs every render (after camera updates)."""
        rgb = self.camera.get_rgb()
        self.last_detection = self._detect(rgb)
    
    def get_detection(self):
        """Called from control loop (may be slightly stale)."""
        return self.last_detection
```

## Episode Loop Template

```python
async def run_training():
    world = World(physics_dt=1/120, rendering_dt=1/60)
    task = MyTask()
    world.add_task(task)
    
    for episode in range(num_episodes):
        # Reset
        await world.reset_async()
        
        # Warm-up
        for _ in range(10):
            world.step(render=False)
        
        # Episode
        done = False
        while not done:
            obs = task.get_observations()
            action = policy(obs)
            task.apply_action(action)
            world.step(render=False)
            done = task.is_done()
        
        # Log results
        log_episode(episode, task.get_metrics())
    
    world.stop()
```

## Chapter Summary

Key patterns:
- Explicitly reset all state after `world.reset()`
- Use Task class for structured scene setup
- Register callbacks in dependency order
- Control at physics rate, perception at render rate
- Warm-up before collecting data

This completes Part V. You now understand Isaac Sim's execution model well enough to debug timing and reproducibility issues.

Next: [Integration Options Overview](../integration/18-integration-overview.md)—connecting Isaac Sim to external systems.

## References

- [Isaac Sim Tasks](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_adding_a_manipulator_robot.html)
- [Isaac Lab Environments](https://isaac-sim.github.io/IsaacLab/main/source/api/isaaclab.envs.html)
