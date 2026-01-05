# Launching Isaac Sim

Get Isaac Sim running in under 30 minutes.

## Installation Options

**Omniverse Launcher (Recommended for beginners)**

1. Download [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)
2. Install Omniverse Launcher
3. Open Launcher → Exchange → Search "Isaac Sim" → Install
4. Launch from Library tab

**pip install (For Python-first workflows)**

```bash
# Requires Python 3.10
pip install isaacsim[all] --extra-index-url https://pypi.nvidia.com
```

**Docker (For CI/headless)**

```bash
# Login to NGC
docker login nvcr.io
# Username: $oauthtoken
# Password: your NGC API key

# Pull and run
docker pull nvcr.io/nvidia/isaac-sim:4.5.0
docker run --gpus all -it nvcr.io/nvidia/isaac-sim:4.5.0
```

## System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | RTX 2070 | RTX 4080+ |
| VRAM | 8 GB | 16+ GB |
| RAM | 32 GB | 64 GB |
| CPU | 8 cores | 16+ cores |
| Storage | 50 GB SSD | 100+ GB NVMe |

Check your NVIDIA driver: `nvidia-smi`. Isaac Sim 4.5 requires driver 535+.

## First Launch

Launch Isaac Sim from Omniverse Launcher. First launch takes several minutes—it's compiling shaders.

You'll see:
- **Viewport**: 3D scene view
- **Stage**: Scene hierarchy (USD prims)
- **Property**: Selected object properties
- **Content**: Asset browser
- **Console**: Logs and Python output

## Run a Built-in Example

1. **File → Open** 
2. Navigate to: `Isaac/Samples/Isaac_Sim_Basics/Tutorials/`
3. Open `franka_basic.usd`
4. Click **Play** (▶️ button or spacebar)

The Franka arm should move through a pre-programmed motion.

## What "Play" Actually Does

When you click Play:
1. Physics engine initializes
2. Simulation time starts advancing
3. Physics steps at fixed intervals (default: 60 Hz)
4. Rendering updates (may be different rate)
5. Any registered callbacks execute

When you click Stop:
- Simulation pauses
- Time resets to 0
- Scene reverts to initial state

This is enough to proceed. The execution model details come in [Part V](../execution/15-simulation-execution-model.md) after you've seen the bugs it causes.

## Troubleshooting First Launch

**Black viewport**: GPU driver issue. Update to latest driver.

**"No GPU found"**: Check `nvidia-smi`. If it fails, reinstall NVIDIA drivers.

**Crash on launch**: Check RAM. Close other applications. Isaac Sim needs 16+ GB free.

**Slow first launch**: Normal. Shader compilation can take 5–10 minutes. Subsequent launches are faster.

## Chapter Summary

You installed Isaac Sim, launched it, and ran a sample scene. You know that Play starts physics and Stop resets the scene.

Next: [Your First Robot](./02-your-first-robot.md)—load a robot and inspect its structure.

## References

- [Isaac Sim Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/index.html)
- [System Requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim Release Notes](https://docs.omniverse.nvidia.com/isaacsim/latest/release_notes.html)
