# Failure Mode Index

Find your symptom → get the cause → go to the chapter.

## Scene/USD Issues

| Symptom | Likely Cause | See |
|---------|--------------|-----|
| Asset doesn't appear | Missing default prim | Ch 5 |
| Wrong scale (100x off) | Unit mismatch | Ch 6 |
| Robot sideways | Up axis mismatch | Ch 6 |
| Reference fails silently | Invalid path | Ch 5 |
| Changes disappear on save | Wrong edit target | Ch 5 |

## Physics Issues

| Symptom | Likely Cause | See |
|---------|--------------|-----|
| Objects explode on start | Interpenetration or scale | Ch 6, 10 |
| Robot doesn't move | Physics not applied | Ch 5 |
| Joints oscillate | Stiffness/timestep mismatch | Ch 16 |
| Objects fall through floor | Missing collision | Ch 10 |
| Different results each run | Non-determinism | Ch 16 |

## Control Issues

| Symptom | Likely Cause | See |
|---------|--------------|-----|
| Variable control rate | Using render callback | Ch 15 |
| Stale sensor data | Querying at wrong time | Ch 15 |
| Reset doesn't reset | Incomplete state reset | Ch 17 |
| Control doesn't take effect | Async delay | Ch 15 |

## Integration Issues

| Symptom | Likely Cause | See |
|---------|--------------|-----|
| ROS time drifts | Time sync issues | Ch 20 |
| Transforms wrong | TF frame mismatch | Ch 20 |
| Slow with ROS | Bridge overhead | Ch 22 |

## Training Issues

| Symptom | Likely Cause | See |
|---------|--------------|-----|
| Policy doesn't learn | Reward design | Ch 28 |
| Training too slow | Not using GPU physics | Ch 29 |
| Results don't reproduce | Seed not set | Ch 16 |
| Policy doesn't transfer | Sim-to-real gap | Ch 35 |
