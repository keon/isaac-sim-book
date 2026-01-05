# How Scenes Are Actually Stored

Your first scenes work. Then you save, reload, and something's wrong. Or you import an asset and it appears at 100x scale. Or you modify a robot and break physics.

This chapter explains USD—just enough to fix these problems.

## What a USD File Actually Is

A `.usd` file is a hierarchical scene graph. Open one in a text editor (`.usda` format):

```usda
#usda 1.0
(
    metersPerUnit = 1.0
    upAxis = "Z"
)

def Xform "World"
{
    def Xform "Robot"
    {
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        
        def Cube "Body"
        {
            double size = 0.5
        }
    }
}
```

Key concepts:
- **Prims**: Things in the scene (`World`, `Robot`, `Body`)
- **Properties**: Data on prims (`xformOp:translate`, `size`)
- **Hierarchy**: Prims contain other prims (like folders)

## Stages and Prims

The **stage** is the root of the scene. Get it via:

```python
import omni.usd
stage = omni.usd.get_context().get_stage()
```

**Prims** are accessed by path:

```python
from pxr import Usd

# Get a prim
robot = stage.GetPrimAtPath("/World/Robot")

# Check if it exists
if robot.IsValid():
    print(f"Found robot: {robot.GetTypeName()}")

# Get children
for child in robot.GetChildren():
    print(child.GetPath())
```

## What Users Accidentally Corrupt

### Problem 1: Missing Default Prim

When you reference a USD file, USD looks for a "default prim" to use as root. Without one, the reference fails silently.

**Check for default prim:**
```python
default_prim = stage.GetDefaultPrim()
if not default_prim:
    print("⚠ No default prim set!")
```

**Fix:**
```python
world = stage.GetPrimAtPath("/World")
stage.SetDefaultPrim(world)
```

### Problem 2: Invalid Prim Paths

Prim paths must:
- Start with `/`
- Use only alphanumeric characters and underscores
- Not contain spaces or special characters

```python
# Good paths
"/World/Robot"
"/World/Shelf_01"
"/World/Object_A"

# Bad paths (will fail)
"World/Robot"     # Missing leading /
"/World/My Robot" # Space
"/World/Shelf-01" # Hyphen
```

### Problem 3: Orphaned Prims

Deleting a parent leaves children orphaned:

```python
# Creates orphaned prims
stage.RemovePrim("/World")  # Children of /World still exist but are invalid

# Correct: remove children first, or use
from omni.kit.commands import execute
execute("DeletePrims", paths=["/World"])  # Handles hierarchy
```

## Transforms

Every prim can have transform operations:

```python
from pxr import UsdGeom, Gf

prim = stage.GetPrimAtPath("/World/Robot")
xform = UsdGeom.Xformable(prim)

# Set position
xform.ClearXformOpOrder()
xform.AddTranslateOp().Set(Gf.Vec3d(1.0, 2.0, 0.5))

# Set rotation (degrees)
xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 45))

# Get world position (includes parent transforms)
cache = UsdGeom.XformCache()
world_transform = cache.GetLocalToWorldTransform(prim)
world_pos = world_transform.ExtractTranslation()
```

## Layers: Why Your Changes Disappear

USD scenes can have multiple **layers**. Changes go to the **edit target layer**—which might not be what you expect.

```python
# Check current edit target
edit_target = stage.GetEditTarget().GetLayer()
print(f"Editing: {edit_target.identifier}")

# Common problem: editing a referenced file
# Your changes go to the reference, not your scene
```

**Fix:** Ensure you're editing the right layer:

```python
root_layer = stage.GetRootLayer()
stage.SetEditTarget(root_layer)
```

## References: Reusing Assets

References pull in external USD files:

```python
# Reference a robot
robot = stage.DefinePrim("/World/Robot", "Xform")
robot.GetReferences().AddReference("./assets/franka.usd")
```

**Common problems:**
- Wrong path (relative vs absolute)
- Missing default prim in referenced file
- Unit mismatch between files

**Check references:**
```python
refs = robot.GetReferences()
for ref in refs.GetAddedOrExplicitItems():
    print(f"References: {ref.assetPath}")
```

## Schemas: What Makes Physics Work

**Schemas** define what properties a prim can have. For physics:

```python
from pxr import UsdPhysics

prim = stage.GetPrimAtPath("/World/Box")

# Check if physics is applied
if prim.HasAPI(UsdPhysics.RigidBodyAPI):
    print("Has rigid body")
if prim.HasAPI(UsdPhysics.CollisionAPI):
    print("Has collision")

# Apply physics to a prim
UsdPhysics.RigidBodyAPI.Apply(prim)
UsdPhysics.CollisionAPI.Apply(prim)
```

**Robot articulations require:**
- `ArticulationRootAPI` on the robot root
- `RigidBodyAPI` on each link
- Joint prims connecting links

## Quick Diagnostics

When things look wrong:

```python
def diagnose_prim(prim_path):
    """Quick diagnostic for a prim."""
    prim = stage.GetPrimAtPath(prim_path)
    
    if not prim.IsValid():
        print(f"❌ Prim not found: {prim_path}")
        return
    
    print(f"✓ Prim: {prim_path}")
    print(f"  Type: {prim.GetTypeName()}")
    print(f"  Active: {prim.IsActive()}")
    
    # Check for common APIs
    apis = ["RigidBodyAPI", "CollisionAPI", "ArticulationRootAPI"]
    for api in apis:
        if prim.HasAPI(getattr(UsdPhysics, api, None)):
            print(f"  Has: {api}")
    
    # Check transform
    if prim.IsA(UsdGeom.Xformable):
        xform = UsdGeom.Xformable(prim)
        for op in xform.GetOrderedXformOps():
            print(f"  Transform: {op.GetOpName()} = {op.Get()}")

diagnose_prim("/World/Robot")
```

## Chapter Summary

USD scenes are hierarchies of prims with properties. When scenes break:
- Check prim paths are valid
- Verify default prim is set
- Confirm edit target is your layer
- Ensure physics schemas are applied

You now know enough USD to fix basic scene problems. Deeper issues (composition, variants) come later when needed.

Next: [Transforms, Units, and Frames](./06-transforms-units-frames.md)—the silent errors that waste hours.

## References

- [OpenUSD Documentation](https://openusd.org/release/index.html)
- [Isaac Sim USD Overview](https://docs.omniverse.nvidia.com/isaacsim/latest/open_usd.html)
