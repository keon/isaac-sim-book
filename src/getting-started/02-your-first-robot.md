# Your First Robot

Load a robot, move its joints, and understand articulation basics.

## Load a Standard Robot

1. **Create → Isaac → Robots → Franka**

   Or drag from Content browser: `Isaac/Robots/Franka/franka.usd`

2. The robot appears at the origin

3. In the Stage panel, expand the robot hierarchy:
   ```
   /World
     /Franka
       /panda_link0
       /panda_link1
       ...
       /panda_hand
   ```

## Inspect the Articulation

Select `/World/Franka` in Stage. In Property panel, find **Articulation Root**.

Key properties:
- **Enabled**: Physics active for this articulation
- **Fix Base**: Whether base link is welded to world

Expand any joint (e.g., `panda_joint1`). You'll see:
- **Type**: Revolute (rotational)
- **Lower/Upper Limit**: Joint range in radians
- **Drive**: How the joint is controlled

## Move Joints Manually

With simulation **stopped**:

1. Select a joint in Stage
2. In Property panel, find **Joint State** or **Drive Target**
3. Drag the position slider
4. Robot moves in viewport

This is direct manipulation—useful for posing, not for control.

## Move Joints During Simulation

1. Click **Play**
2. Open **Isaac Utils → Joint Commander**
3. Select the Franka articulation
4. Drag joint sliders

The robot moves smoothly because physics interpolates between targets.

## Understanding Joint Drives

Isaac Sim joints are controlled by **drives**—virtual motors with:

- **Target Position**: Where the joint should go
- **Target Velocity**: How fast it should move
- **Stiffness**: How hard it tries to reach position (like a spring)
- **Damping**: How much it resists velocity (like friction)

High stiffness + low damping = stiff position control
Low stiffness + high damping = compliant motion

The default Franka has reasonable values. Tuning comes in [Articulations and Drives](../robot-modeling/09-articulations-drives.md).

## Inspect Articulation State

With simulation running, check the Console for articulation state:

```python
# In Script Editor (Window → Script Editor)
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

world = World.instance()
franka = Articulation("/World/Franka")
franka.initialize()

# Get current joint positions (radians)
print(franka.get_joint_positions())

# Get joint velocities
print(franka.get_joint_velocities())
```

## Try Different Robots

Load other robots from Content browser:

- `Isaac/Robots/Unitree/` — quadrupeds
- `Isaac/Robots/Carter/` — wheeled mobile base
- `Isaac/Robots/UR10/` — industrial arm

Each has different joint configurations. Inspect their articulation properties.

## Chapter Summary

You loaded a robot, inspected its joint structure, and moved joints both manually and during simulation. You understand that joints have drives with stiffness and damping.

Next: [First Python Control](./03-first-python-control.md)—control the robot programmatically.

## References

- [Adding Robots](https://docs.omniverse.nvidia.com/isaacsim/latest/features/robots_simulation/index.html)
- [Articulation API](https://docs.omniverse.nvidia.com/py/isaacsim/source/isaacsim.core/isaacsim.core.articulations.html)
