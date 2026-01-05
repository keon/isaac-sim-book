# Minimal USD Reference

Quick reference for USD concepts used in this book.

## Stage and Prims

```python
import omni.usd
from pxr import Usd, UsdGeom

# Get stage
stage = omni.usd.get_context().get_stage()

# Get prim
prim = stage.GetPrimAtPath("/World/Robot")

# Create prim
new_prim = stage.DefinePrim("/World/Box", "Cube")

# Delete prim
stage.RemovePrim("/World/Box")

# Traverse
for prim in stage.Traverse():
    print(prim.GetPath())
```

## Transforms

```python
from pxr import UsdGeom, Gf

xform = UsdGeom.Xformable(prim)

# Set transform
xform.ClearXformOpOrder()
xform.AddTranslateOp().Set(Gf.Vec3d(1, 2, 3))
xform.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 90))
xform.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))

# Get world transform
cache = UsdGeom.XformCache()
world_transform = cache.GetLocalToWorldTransform(prim)
```

## Stage Settings

```python
# Units
UsdGeom.GetStageMetersPerUnit(stage)  # 1.0 = meters
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# Up axis
UsdGeom.GetStageUpAxis(stage)  # "Y" or "Z"
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
```

## Physics Schemas

```python
from pxr import UsdPhysics

# Check/apply APIs
prim.HasAPI(UsdPhysics.RigidBodyAPI)
UsdPhysics.RigidBodyAPI.Apply(prim)
UsdPhysics.CollisionAPI.Apply(prim)
UsdPhysics.ArticulationRootAPI.Apply(prim)
```

## References

```python
# Add reference
prim.GetReferences().AddReference("./asset.usd")

# Add payload
prim.GetPayloads().AddPayload("./heavy_asset.usd")

# Load/unload payloads
stage.Load("/World/Environment")
stage.Unload("/World/Environment")
```

## Layers

```python
# Get layers
root_layer = stage.GetRootLayer()
sublayers = root_layer.subLayerPaths

# Set edit target
stage.SetEditTarget(root_layer)
```
