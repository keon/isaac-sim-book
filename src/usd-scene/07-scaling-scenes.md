# Scaling Scenes Without Pain

Small test scenes load instantly. Add 1000 objects and your scene takes minutes to load, uses 40GB of RAM, and runs at 2 FPS.

This chapter covers techniques to scale: references for reuse, instancing for performance, layers for organization.

## References: Reuse Without Copying

Don't duplicate assets—reference them:

```python
# Bad: loads entire file into scene
stage.DefinePrim("/World/Robot1", "Xform")
# ... copy all robot prims manually

# Good: reference the file
robot = stage.DefinePrim("/World/Robot", "Xform")
robot.GetReferences().AddReference("./assets/franka.usd")
```

**Multiple instances of same asset:**
```python
for i in range(5):
    robot = stage.DefinePrim(f"/World/Robot_{i}", "Xform")
    robot.GetReferences().AddReference("./assets/franka.usd")
    
    # Each can have unique transform
    xform = UsdGeom.Xformable(robot)
    xform.AddTranslateOp().Set(Gf.Vec3d(i * 2.0, 0, 0))
```

## Instancing: 1000x Memory Savings

For many identical objects (boxes, pallets), use **Point Instancer**:

```python
from pxr import UsdGeom, Gf, Vt

# Create point instancer
instancer = UsdGeom.PointInstancer.Define(stage, "/World/Boxes")

# Define prototype (the object to copy)
box_proto = stage.DefinePrim("/World/Boxes/Prototypes/Box", "Cube")
UsdGeom.Cube(box_proto).CreateSizeAttr(0.3)
instancer.CreatePrototypesRel().AddTarget("/World/Boxes/Prototypes/Box")

# Create 1000 instances
positions = []
for i in range(1000):
    x = (i % 10) * 0.4
    y = (i // 10 % 10) * 0.4
    z = (i // 100) * 0.4
    positions.append(Gf.Vec3f(x, y, z))

instancer.CreatePositionsAttr(positions)
instancer.CreateProtoIndicesAttr([0] * 1000)  # All use prototype 0
```

**Result**: 1000 boxes using memory of 1 box.

**Limitation**: Point instancers don't support physics. For physics objects, create individual prims but share collision geometry via references.

## Layers: Keep Scenes Organized

Split your scene into layers:

```
my_scene.usd (root)
├── environment.usd (sublayer) - static geometry
├── lighting.usd (sublayer) - lights
├── robots.usd (sublayer) - robot configurations
└── overrides.usd (sublayer) - project-specific changes
```

**Create layer structure:**
```python
from pxr import Sdf, Usd

# Create root layer
root = Sdf.Layer.CreateNew("my_scene.usd")

# Add sublayers (later = stronger opinions)
root.subLayerPaths = [
    "./environment.usd",
    "./lighting.usd",
    "./robots.usd",
    "./overrides.usd",
]

root.Save()
```

**Benefits:**
- Teams work on different layers simultaneously
- Swap environments without affecting robots
- Override properties without modifying source files

## Payloads: Load Only What You Need

Heavy assets should be **payloads** (deferred loading):

```python
# Create payloaded environment
env = stage.DefinePrim("/World/Environment", "Xform")
env.GetPayloads().AddPayload("./warehouse_detailed.usd")
```

**Control loading:**
```python
# Open with nothing loaded
stage = Usd.Stage.Open("scene.usd", Usd.Stage.LoadNone)

# Load specific parts
stage.Load("/World/Environment")

# Unload to free memory
stage.Unload("/World/Environment")

# Check what's loaded
for prim in stage.Traverse():
    if prim.HasPayload():
        print(f"{prim.GetPath()}: {'loaded' if prim.IsLoaded() else 'unloaded'}")
```

## Collision Simplification

Visual geometry is too detailed for physics. Use simple collision:

```python
from pxr import UsdPhysics

# Visual mesh: 50,000 triangles
visual = stage.GetPrimAtPath("/World/Shelf/VisualMesh")

# Collision: box approximation
collision = UsdGeom.Cube.Define(stage, "/World/Shelf/CollisionBox")
collision.CreateSizeAttr(1.0)

xform = UsdGeom.Xformable(collision.GetPrim())
xform.AddScaleOp().Set(Gf.Vec3f(2.0, 0.5, 1.5))  # Scale to match visual bounds

# Make collision invisible
collision.CreatePurposeAttr(UsdGeom.Tokens.guide)

# Apply collision API
UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
```

## SimReady Assets

NVIDIA provides pre-configured assets with proper physics:

```python
from omni.isaac.core.utils.nucleus import get_assets_root_path

assets = get_assets_root_path()
shelf = f"{assets}/Isaac/Environments/Simple_Warehouse/Props/SM_RackLarge_01.usd"

prim = stage.DefinePrim("/World/Shelf", "Xform")
prim.GetReferences().AddReference(shelf)
```

SimReady assets include:
- Correct collision geometry
- Mass properties
- Physics materials

## Procedural Generation

Generate environments programmatically:

```python
def generate_warehouse(stage, rows, cols, aisle_width=3.0):
    """Generate a grid of shelving units."""
    
    shelf_path = get_assets_root_path() + "/Isaac/Environments/Simple_Warehouse/Props/SM_RackLarge_01.usd"
    
    for row in range(rows):
        for col in range(cols):
            shelf = stage.DefinePrim(f"/World/Shelves/Shelf_R{row}_C{col}", "Xform")
            shelf.GetReferences().AddReference(shelf_path)
            
            x = col * aisle_width
            y = row * 4.0  # Shelf depth + gap
            
            xform = UsdGeom.Xformable(shelf)
            xform.AddTranslateOp().Set(Gf.Vec3d(x, y, 0))

# Generate 5x10 warehouse
generate_warehouse(stage, rows=5, cols=10)
```

## Performance Checklist

Before running slow scenes:

- [ ] Using point instancers for identical static objects?
- [ ] Collision geometry simplified?
- [ ] Heavy assets are payloads?
- [ ] Unnecessary layers unloaded?
- [ ] Render resolution appropriate?

## Chapter Summary

Scale scenes by:
- **References**: Reuse assets without copying
- **Point instancers**: 1000x memory savings for identical objects
- **Layers**: Organize and parallelize work
- **Payloads**: Load only what's needed
- **Simple collision**: Don't simulate visual detail

Next: [Importing Robots](../robot-modeling/08-importing-robots.md)—get your robot into Isaac Sim correctly.

## References

- [USD Point Instancers](https://openusd.org/release/api/class_usd_geom_point_instancer.html)
- [Isaac Sim Assets](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/assets.html)
