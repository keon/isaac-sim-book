# Transforms, Units, and Frames

Your robot is 100x too big. Or sideways. Or physics explodes immediately. These are unit and frame mismatches—the most common silent errors in Isaac Sim.

## The Problem

Isaac Sim expects:
- **Units**: Meters
- **Up axis**: Z-up
- **Coordinate system**: Right-handed

Assets from other tools often differ:
- Blender: Centimeters (0.01 scale), Z-up
- Maya: Centimeters, Y-up
- CAD software: Millimeters (0.001 scale), varies
- URDF: Meters, Z-up (usually compatible)

When these don't match, things go wrong silently.

## Check Stage Settings

```python
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()

# What units is this stage using?
meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
print(f"Meters per unit: {meters_per_unit}")
# 1.0 = meters, 0.01 = centimeters, 0.001 = millimeters

# What's "up"?
up_axis = UsdGeom.GetStageUpAxis(stage)
print(f"Up axis: {up_axis}")  # "Y" or "Z"
```

## Fix 1: Robot is Huge or Tiny

**Symptom**: Robot appears 100x or 1000x wrong size.

**Diagnose**:
```python
from pxr import UsdGeom

prim = stage.GetPrimAtPath("/World/Robot")
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
bbox = bbox_cache.ComputeWorldBound(prim)
size = bbox.ComputeAlignedRange().GetSize()
print(f"Size: {size}")  # Should be reasonable (e.g., 0.5m for a robot arm)
```

**Fix options**:

1. **Scale on reference** (recommended):
```python
from pxr import Gf

robot = stage.DefinePrim("/World/Robot", "Xform")
robot.GetReferences().AddReference("./robot_in_cm.usd")

# Scale from cm to m
xform = UsdGeom.Xformable(robot)
xform.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
```

2. **Fix source file**: Re-export with correct units.

3. **Set stage metadata** (if everything is consistently wrong):
```python
UsdGeom.SetStageMetersPerUnit(stage, 0.01)  # Tell Isaac it's in cm
```

## Fix 2: Robot is Sideways

**Symptom**: Robot lies flat or is rotated 90°.

**Cause**: Y-up file loaded into Z-up stage.

**Fix**:
```python
robot = stage.DefinePrim("/World/Robot", "Xform")
robot.GetReferences().AddReference("./robot_y_up.usd")

xform = UsdGeom.Xformable(robot)
# Rotate 90° around X to convert Y-up to Z-up
xform.AddRotateXOp().Set(-90.0)
```

## Fix 3: Physics Explodes

**Symptom**: Objects fly apart on first frame.

**Causes**:
- Collision geometry at wrong scale (objects interpenetrating)
- Mass inconsistent with size

**Diagnose**:
```python
from pxr import UsdPhysics

def check_physics_object(prim):
    # Check collision bounds
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        bbox = bbox_cache.ComputeWorldBound(prim)
        size = bbox.ComputeAlignedRange().GetSize()
        print(f"Collision size: {size}")
    
    # Check mass
    mass_api = UsdPhysics.MassAPI(prim)
    if mass_api:
        mass = mass_api.GetMassAttr().Get()
        print(f"Mass: {mass}")
```

**Fix**: Ensure collision geometry and mass use consistent units.

## Camera Frames

Isaac Sim cameras use OpenGL convention:
- **-Z**: Forward (into scene)
- **+Y**: Up
- **+X**: Right

If your camera points the wrong way:
```python
from pxr import UsdGeom, Gf

camera = stage.GetPrimAtPath("/World/Camera")
xform = UsdGeom.Xformable(camera)

# Make camera look along robot's +X axis
xform.ClearXformOpOrder()
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))  # Position
xform.AddRotateYOp().Set(90)  # Rotate so -Z points along +X
```

## Validation Script

Run this to check for common issues:

```python
def validate_scene():
    """Check for unit and frame issues."""
    stage = omni.usd.get_context().get_stage()
    issues = []
    
    # Stage settings
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    
    if mpu != 1.0:
        issues.append(f"Stage not in meters (metersPerUnit={mpu})")
    if up != UsdGeom.Tokens.z:
        issues.append(f"Stage not Z-up (upAxis={up})")
    
    # Check object sizes
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
        bbox = bbox_cache.ComputeWorldBound(prim)
        size = bbox.ComputeAlignedRange().GetSize()
        max_dim = max(size)
        
        if max_dim < 0.001:
            issues.append(f"{prim.GetPath()}: Very small ({max_dim:.6f}m)")
        elif max_dim > 100:
            issues.append(f"{prim.GetPath()}: Very large ({max_dim:.1f}m)")
    
    if issues:
        for issue in issues:
            print(f"⚠ {issue}")
    else:
        print("✓ No unit/frame issues detected")

validate_scene()
```

## Metrics Assembler

Isaac Sim has automatic unit conversion called **Metrics Assembler**. It tries to fix unit mismatches when referencing files.

**When it helps**: Importing assets with correct metadata but different units.

**When it fails**: Assets with wrong metadata (claims meters, is actually cm).

**To disable** (for manual control):
```
Edit → Preferences → Stage → Metrics Assembler: Off
```

## Chapter Summary

Most "weird physics" bugs are unit/frame mismatches:
- Check `metersPerUnit` and `upAxis` on your stage
- Apply scale corrections when referencing non-meter assets
- Apply rotation corrections for Y-up assets
- Validate collision geometry sizes
- Remember camera uses -Z forward

Next: [Scaling Scenes Without Pain](./07-scaling-scenes.md)—references, instancing, and layers.

## References

- [Isaac Sim Conventions](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/reference_conventions.html)
- [OpenUSD Transform Tutorial](https://openusd.org/release/tut_xforms.html)
