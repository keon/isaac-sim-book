# Version Compatibility

## Isaac Sim Versions

| Isaac Sim | Isaac Lab | Python | Driver |
|-----------|-----------|--------|--------|
| 4.5.0 | 2.0.x | 3.10 | 535+ |
| 4.2.0 | 1.2.x | 3.10 | 525+ |
| 4.0.0 | 1.0.x | 3.10 | 525+ |
| 2023.1.1 | 0.6.x | 3.10 | 525+ |

## Common Breaking Changes

### 4.5.0
- API changes in articulation interface
- New camera sensor API

### 4.2.0
- Isaac Lab renamed from Orbit
- Environment API changes

### 4.0.0
- Major PhysX upgrade
- Scene structure changes

## Checking Versions

```python
# Isaac Sim version
import carb
print(carb.settings.get_settings().get("/app/version"))

# Python version
import sys
print(sys.version)

# Driver version
# Run: nvidia-smi
```

## Migration Tips

1. Read release notes before upgrading
2. Test in isolation first
3. Check API changes for your imports
4. Validate physics behavior
5. Re-run regression tests
