Drop STL files here. Reference them from a URDF `<visual>` as
`meshes/<file>` (relative to this folder), e.g.

```xml
<mesh filename="meshes/upper_arm.stl" scale="1 1 1"/>
```

Units: meters (or scale accordingly). Binary or ASCII STL both work.

`sample_cube.stl` (0.1 m, binary) is a known-good reference — the
`test_mesh.urdf` model renders it; check it at
`http://127.0.0.1:8081/?model=test_mesh`. If your first STL does not
show up, compare against this one.
