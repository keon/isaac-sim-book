# How to Use This Book

This book teaches NVIDIA Isaac Sim through practice first, theory when needed.

## Reader Paths

**Roboticists** integrating with ROS 2 or existing stacks:
- Start with Parts I–IV (getting running, scenes, robots, sensors)
- Jump to Part VI (Integration) when ready
- Return to Part V (Execution) when debugging timing issues

**ML Engineers** training policies or generating synthetic data:
- Start with Parts I–III (getting running, scenes, robots)
- Jump to Part VII (Synthetic Data) or Part VIII (Learning)
- Return to Part V when debugging reproducibility

**Graphics/Simulation Engineers** from USD/Omniverse backgrounds:
- Skim Part I, dive deep into Part II
- Part V will clarify execution model differences from other DCC tools

## What You Can Skip

- **Part II** (USD) if your scenes already work
- **Part V** (Execution) until you hit timing or reproducibility bugs
- **Part VI** (Integration) if not using ROS 2 or external systems
- **Part VII–VIII** if not doing synthetic data or RL

## Running Example

Throughout this book, we build a **mobile manipulator in a warehouse**:
- A differential-drive base with a 6-DOF arm
- Operating in a facility with shelves, lighting variations, and dynamic obstacles
- Tasks: navigation, perception, pick-and-place

Each part adds capability:
1. **Start Running**: Load and move the robot
2. **Scenes**: Build the warehouse environment
3. **Robots**: Configure stable articulation and contact
4. **Sensors**: Add cameras, LiDAR, force sensing
5. **Execution**: Understand timing and reproducibility
6. **Integration**: Connect to ROS 2 navigation
7. **Synthetic Data**: Generate training images
8. **Learning**: Train a grasping policy
9. **End-to-End**: Full pick operation
10. **Production**: Reproducible training, sim-to-real transfer

## Repository Structure

```
isaac-book-code/
├── checkpoints/          # Saved scenes after each part
│   ├── part1-basics/
│   ├── part2-warehouse/
│   └── ...
├── scripts/              # Python scripts by chapter
├── configs/              # Configuration files
└── assets/               # Custom meshes and materials
```

Each chapter references specific checkpoint files. Start from any checkpoint if you want to skip ahead.

## Conventions

- **Code blocks** are tested against Isaac Sim 4.5 / Isaac Lab 2.0
- **Shell commands** assume Linux; Windows equivalents noted where different
- **Paths** use forward slashes; substitute as needed
- **"Play"** means clicking the Play button or calling `world.reset()` + `world.step()`
