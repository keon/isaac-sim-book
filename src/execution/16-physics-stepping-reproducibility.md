# Physics Stepping and Reproducibility

You run the same simulation twice and get different results. Your regression tests fail randomly. Training results can't be reproduced.

This chapter explains what determinism Isaac Sim actually provides—and how to work within those limits.

## What Determinism Means

**Determinism**: Same inputs → same outputs.

Isaac Sim (via PhysX) provides **conditional determinism**: determinism under specific conditions.

## What IS Guaranteed

When running on:
- Same GPU model
- Same driver version
- Same Isaac Sim version
- Same simulation parameters
- Same initial state
- Same input sequence

...you get **bit-identical results**.

This is enough for:
- Debugging (reproduce exact failures)
- Regression testing (same hardware in CI)
- Reproducible experiments (with environment control)

## What IS NOT Guaranteed

Determinism breaks across:
- **Different GPU models**: RTX 4090 ≠ RTX 3080
- **Different driver versions**: Updates change floating-point behavior
- **CPU vs GPU physics**: Different code paths
- **Different operating systems**: Floating-point varies
- **Soft body/cloth**: These solvers aren't deterministic

## Why Results Differ

### Floating-Point Arithmetic

GPUs run operations in parallel. Order affects results:

```
(a + b) + c ≠ a + (b + c)  # In floating-point
```

Different GPUs schedule differently, changing summation order.

### Contact Order

When multiple objects collide, PhysX processes them in some order. Thread scheduling differences change this order.

### Random Seeds

Isaac Lab seed control:

```python
from omni.isaac.lab.utils.seed import set_seed
set_seed(42)  # Sets Python, NumPy, PyTorch, PhysX seeds
```

But seeds don't overcome hardware floating-point differences.

## Practical Strategies

### Strategy 1: Lock Down Environment

For true determinism, use containers:

```dockerfile
FROM nvcr.io/nvidia/isaac-sim:4.5.0
# Same GPU model in CI
# Same driver version
# Identical results
```

### Strategy 2: Statistical Reproducibility

For training, report statistics instead of exact values:

```python
seeds = [1, 2, 3, 4, 5]
results = [train_policy(seed=s) for s in seeds]
print(f"Reward: {np.mean(results):.2f} ± {np.std(results):.2f}")
```

Claim "achieves reward X ± Y" not "achieves exactly X with seed 42."

### Strategy 3: Robust Control Design

Feedback control tolerates variation:

```python
# Open-loop (fragile)
for t, position in trajectory:
    robot.set_position(position)  # Breaks if physics differs

# Closed-loop (robust)
def control(state):
    error = target - state
    return kp * error + kd * error_derivative  # Corrects deviations
```

### Strategy 4: Statistical Tests in CI

Don't check exact values—check bounds:

```python
def test_robot_reaches_target():
    results = [run_simulation(seed=s) for s in range(5)]
    mean_error = np.mean([np.linalg.norm(r - target) for r in results])
    assert mean_error < 0.1, f"Mean error {mean_error} too high"
```

## Timestep Selection

The physics timestep affects stability and accuracy.

### Guidelines

| Scenario | Timestep | Why |
|----------|----------|-----|
| Mobile navigation | 1/60 s | Slow dynamics |
| Manipulation | 1/120 s | Contact events |
| Fast manipulation | 1/240 s | Stiff contacts |
| Legged locomotion | 1/200 s | Impact events |

### Stability Rule

Objects shouldn't move more than their size per step:

```python
max_velocity = 10  # m/s
object_size = 0.1  # m
min_rate = max_velocity / object_size  # 100 Hz
```

If objects tunnel through each other, reduce timestep.

### Joint Stiffness

High stiffness needs small timestep:

```
dt_stable ≈ 2 * sqrt(mass / stiffness)
```

Oscillating joints → reduce timestep or reduce stiffness.

## Solver Parameters

PhysX solver is iterative:

```python
# Increase for better constraint resolution
position_iterations = 12  # Default 8
velocity_iterations = 4   # Default 1
```

Higher iterations = more accurate but slower.

Manipulation (grasping) often needs higher iterations than locomotion.

## GPU vs CPU Physics

**GPU physics**:
```python
physics_context.enable_gpu_dynamics(True)
```
- Fast for parallel environments
- Essential for RL training
- Different numerical results than CPU

**CPU physics**:
```python
physics_context.enable_gpu_dynamics(False)
```
- More predictable
- Better debugging
- Slow for parallel envs

**Development workflow**: Debug on CPU, validate on GPU, train on GPU.

## Common Instabilities

### Exploding Joints

**Symptoms**: Joint values grow unbounded, robot flies apart.

**Causes**:
- Timestep too large for stiffness
- Solver iterations too low
- Initial interpenetration

**Fixes**:
- Reduce timestep
- Reduce drive stiffness
- Increase solver iterations
- Check initial configuration

### Contact Oscillation

**Symptoms**: Stacked objects vibrate.

**Causes**:
- Solver can't converge
- Objects interpenetrating

**Fixes**:
- Increase solver iterations
- Reduce restitution
- Add damping

### Gain Wind-Up

**Symptoms**: Integral controller goes to infinity.

```python
class PIController:
    def compute(self, error, dt):
        self.integral += error * dt  # Unbounded!
        return self.kp * error + self.ki * self.integral
```

**Fix**: Clamp integral term.

## Chapter Summary

Key reproducibility facts:
- Same hardware + software + config = deterministic
- Different GPUs = different results
- Use feedback control (tolerates variation)
- Use statistical tests in CI
- Match timestep to your dynamics

Next: [Python Control Patterns](./17-python-control-patterns.md)—resets, tasks, and state management.

## References

- [Isaac Lab Reproducibility](https://isaac-sim.github.io/IsaacLab/main/source/features/reproducibility.html)
- [PhysX Simulation](https://nvidia-omniverse.github.io/PhysX/physx/5.4.2/docs/Simulation.html)
- [PhysX GPU Rigid Bodies](https://nvidia-omniverse.github.io/PhysX/physx/5.4.2/docs/GPURigidBodies.html)
