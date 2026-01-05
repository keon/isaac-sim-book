# Performance Budgets

Typical costs for common operations.

## Physics Stepping

| Configuration | Typical Speed |
|---------------|---------------|
| Single robot, simple scene | 1000+ Hz |
| Single robot, complex scene | 200-500 Hz |
| 100 parallel envs (GPU) | 50-100k steps/sec |
| 1000 parallel envs (GPU) | 200-500k steps/sec |

## Sensors

| Sensor | Typical Cost |
|--------|--------------|
| Joint state | ~0 (free) |
| IMU | <0.1 ms |
| Camera 640x480 RGB | 2-5 ms |
| Camera 1920x1080 RGB | 5-15 ms |
| Depth + segmentation | +2-5 ms |
| LiDAR (RTX) | 3-10 ms |
| LiDAR (ray-based) | 1-3 ms |

## Memory

| Component | Typical Usage |
|-----------|---------------|
| Isaac Sim base | 8-12 GB VRAM |
| Per parallel env | 10-50 MB VRAM |
| Per camera | 50-200 MB VRAM |
| Complex scene | +2-8 GB VRAM |

## Guidelines

- **Training**: Skip rendering when possible
- **Cameras**: Use lowest resolution that works
- **Physics rate**: Use highest timestep that's stable
- **Parallel envs**: Start with 256-1024, scale up as needed
