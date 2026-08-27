# robot_description

Robot description assets for the pick_ik tools. The **web demo viewer**
(`ik_service`, `GET /`) renders whatever is in this folder — no code
changes needed when you add or edit files here.

```
robot_description/
├── arm7.urdf           # 7-DOF arm: joints + limits + primitive visuals
├── test_mesh.urdf      # 2-joint STL smoke test (?model=test_mesh)
├── meshes/             # STL files referenced by <mesh filename="…">
│   ├── README.md
│   └── sample_cube.stl # 0.1 m binary STL example (known-good reference)
└── README.md
```

## Serving

The service mounts this folder read-only at:

```
GET /model/{path}          e.g. /model/arm7.urdf, /model/meshes/part.stl
```

(`ik_service/service/app.py`, route `model_asset`; the same
path-traversal guard as `/lib/*`.)

## How the viewer uses it

The 3D view loads `/model/{model}.urdf` — default model is `arm7`,
override with the URL parameter:

```
http://127.0.0.1:8081/?model=arm7
http://127.0.0.1:8081/?model=test_mesh   # STL smoke test (sample cube)
```

The viewer then:

1. Parses the URDF (DOMParser) — links, joints (revolute/fixed),
   `<visual>` elements with `box` / `sphere` / `cylinder` / `cone`
   geometry or `<mesh>`, and `<material><color rgba>`.
2. Fetches every referenced STL (binary **or** ASCII; units: meters;
   `scale` attribute honored).
3. Each frame, places link *i* at the *i*-th `POST /fk` frame
   (mapping documented in the URDF header) and draws its visuals.

The target ring and ground grid are viewer chrome, not part of the
model.

## Adding real STL files

Put the STL in `meshes/` and reference it in a link's visual:

```xml
<link name="link2">
  <visual>
    <origin xyz="0 -0.20 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="meshes/upper_arm.stl" scale="1 1 1"/>
    </geometry>
    <material name="link_blue"/>
  </visual>
</link>
```

Notes:

* **Units must be meters** (the FK frames are meters). Exporters that
  emit mm: either export in meters or use `scale="0.001 0.001 0.001"`.
* The STL's frame is placed at the `<origin>` (xyz + rpy, rpy =
  fixed-axis `Rz(yaw)·Ry(pitch)·Rx(roll)`, same convention as joints).
* **Winding:** p5 culls back faces. The viewer checks the STL's stored
  normals against the vertex winding and flips triangles if the
  majority is inconsistent, so inward-wound STLs still render.
* **Shading:** this p5 build renders custom triangle meshes with
  flat ambient shading only (no per-face directional light). Meshes
  are clearly visible but do not have the same shading as the p5
  primitives.
* **Size:** keep meshes modest (≲ 50k triangles) — the p5 vertex
  pipeline re-submits mesh geometry every frame.
* Multiple `<visual>` elements per link are supported (the arm7
  tool_link uses one for the knob and one per cross bar).

## Solver boundary (important)

The URDF drives the **visuals** and documents joint origins/limits,
but the **solver's kinematics stay hardcoded in the C++ core**
(`libpick_ik_core/tests/arm7_fk.hpp` — joint table, limits,
tool offset). If you change joint origins/limits in the URDF, the
drawn robot changes but the solver does **not** follow. Keep the two
in sync; a URDF whose chain no longer matches `arm7_fk.hpp` will
render in a pose that does not correspond to the solved q.

(Sooner or later the core can be made URDF-driven; until then treat
`arm7_fk.hpp` as the source of truth for the kinematics and this
folder as the source of truth for the looks.)
