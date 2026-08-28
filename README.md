# ik_service — development interface for libpick_ik_core

A **FastAPI transport layer** over the C++ `pickik` binding (the `IkSolver`
contract in `libpick_ik_core`). The service contains **no solver code** — it
only exposes the C++ solvers over HTTP and owns the arm7 model (Python
reference FK + joint limits).

```
p5.js / Python / browser / PyBullet / IsaacSim / Blender / robot
                      │  HTTP + JSON
                      ▼
             FastAPI service (this directory)
                      │  `pickik` binding (pybind11)
                      ▼
              libpick_ik_core (C++, all solving happens here)
              CcdSolver / PickIkGradientSolver / PickIkMemeticSolver
                      │
                      ▼
                    FK (Python callback in service/arm7.py)
```

## Getting started (fresh machine)

1. **Prereqs:** Python 3.10+ (dev machine uses MS-Store CPython 3.12.28),
   CMake ≥ 3.22, a C++17 compiler (dev machine: Visual Studio 2022 / MSVC),
   and either network access (FetchContent) or a `../.deps` dependency cache.
2. **Clone both repos as siblings** (this repo builds `../libpick_ik_core`):
   ```sh
   git clone https://github.com/SwannSchilling/libpick-ik-core.git
   git clone https://github.com/SwannSchilling/ik-service.git
   ```
3. **Build, test, run:**
   ```sh
   cd ik_service
   pip install -r requirements.txt
   cmake -S . -B build -G "Visual Studio 17 2022" -A x64
   cmake --build build --config RelWithDebInfo   # -> dist/pickik.pyd
   pytest tests/ -q -p no:cacheprovider          # 44 tests
   python -m service.main                        # http://127.0.0.1:8081/
   ```

The `HANDOVER.md` in this repo is the machine-agnostic project resume — read
it when picking the project up.

## Layout

```
ik_service/
    CMakeLists.txt        builds the pickik binding into ./dist/
    requirements.txt      fastapi, uvicorn, numpy, pybind11, pytest, httpx
    _bootstrap.py         sys.path setup: vendor/ (Starlette 0.41.2) + dist/
    vendor/starlette/     MIT, 0.41.2 — shadows system Starlette 1.x (see vendor/README.md)
    service/
        __init__.py
        arm7.py           pure-Python arm7 FK + Robot + limits (the model)
        app.py            FastAPI app (endpoints + demo page)
        main.py           uvicorn runner
    web/index.html        demo: solver dropdown + target + URDF-driven 3D view
    tests/                pytest: FK anchors, binding, API (in-process client)
    dist/                 pickik.pyd lands here after the CMake build
```

## Building

Python deps (already satisfied in this workspace):

```sh
pip install -r requirements.txt
```

The C++ binding (from this directory). The `FETCHCONTENT_SOURCE_DIR_*`
overrides are optional and machine-local: the `../.deps` cache is **not part
of this repo**; where it is absent, FetchContent clones Eigen/fmt/pybind11
from the network instead:

```sh
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 \
  -DFETCHCONTENT_SOURCE_DIR_EIGEN=../.deps/eigen \
  -DFETCHCONTENT_SOURCE_DIR_FMT=../.deps/fmt \
  -DFETCHCONTENT_SOURCE_DIR_PYBIND11=../.deps/pybind11
cmake --build build --config RelWithDebInfo
# -> dist/pickik.pyd
```

(If you have `pip install pybind11` available, `-Dpybind11_DIR="$(python -m
pybind11 --cmake-dir)"` works instead of the pybind11 fetch override.)

`_bootstrap.py` puts both `vendor/` (Starlette 0.41.2, shadowing the
system Starlette 1.x that breaks FastAPI 0.115) and `dist/` (the `pickik`
C++ binding) on `sys.path`. It is imported by `tests/conftest.py` and
`service/main.py`, so both entry points work without extra setup.

## Running

```sh
python -m service.main
# demo UI:  http://127.0.0.1:8081/
# OpenAPI:  http://127.0.0.1:8081/docs
```

Port 8000 is inside a Windows excluded TCP range on this machine, so the
default is 8081; override with `IK_SERVICE_PORT` if needed.

## API

### `POST /solve`

```json
{
  "solver": "gradient",                 // "ccd" | "gradient" | "memetic"
  "seed": [0, -0.4, 0.1, -1.2, 0, 1.3, 0.4],   // optional, rad
  "target": {
    "position": [0.35, -0.05, 0.59],     // meters
    "quaternion": [0.707, 0, 0, 0.707]   // optional [x,y,z,w]; omit = position-only
  },
  "options": { }                        // see below
}
```

```json
{
  "success": true,
  "q": [0.21, -0.53, ...],
  "position_error": 0.00021,            // m
  "orientation_error": 0.00043,         // rad; -1 for position-only
  "solver": "gradient",
  "time_ms": 1.8
}
```

- Default seed = the p5 POC's quantized "all zero" slider state
  (`[-0.00159265, 0, ...]`, documented in `service/arm7.py`).
- Position-only goals: the app sets `orientation_threshold=None` and
  `rotation_scale=0.0` automatically. (If you supply a quaternion, both are
  on: `orientation_threshold=1e-3`, `rotation_scale=0.5` — the contract
  requires the orientation to leave the cost for position-only goals.)
- `options`: `position_threshold`, `orientation_threshold`, `cost_threshold`,
  `position_scale`, `rotation_scale` (generic) and `max_passes`/`damping`
  (ccd), `max_time`/`max_iterations` (gradient), `num_threads`/`max_time`
  (memetic).

### `POST /fk`

```json
{ "q": [0, 0, 0, 0, 0, 0, 0] }
->
{ "frames": [ ...8 4x4 frames... ],
  "tool0": { "position_m": [0,0,1.266], "position_mm": [0,0,1266] } }
```

Useful for p5.js readouts and for plotting; the web demo drives its 2D view
from this endpoint.

### `GET /solvers`, `GET /health`, `GET /`

Solver list, liveness, and the demo page.

## Notes / gotchas

- **Threading**: the memetic solver evaluates FK from worker threads; a
  Python FK callback is serialized through the GIL pump, so extra
  `num_threads` only add queue traffic — the service default is 1 (see
  `libpick_ik_core/docs/integration-roadmap.md` §2.4). Native FK hosts
  (no Python callback) should raise it.
- **Time budgets**: the gradient search is wall-clock bounded (`max_time`);
  the deep-fold target A needs a wide budget (defaults here: 2 s / 2000 iter).
- **p5.js**: fetch the endpoint exactly as in the demo page; the p5 sketch
  itself stays untouched — it can keep its native JS CCD or call this
  service for any solver.
- **Model changes**: geometry/limits live in `libpick_ik_core`'s arm7 tables
  and `ARM7_KINEMATIC_SPEC.md`; `service/arm7.py` must stay in sync
  (tests pin it to the spec's anchor poses).

## Tests

```sh
pytest tests/ -q -p no:cacheprovider   # cacheprovider off: this workspace denies cache-file writes
```

- `test_arm7_fk.py` — Python FK vs. the nine verified anchor poses
  (ARM7_KINEMATIC_SPEC.md §5) + frame-contract checks.
- `test_binding.py` — binding surface + all three solvers on the p5
  cross-check targets, error paths, numpy-vs-list FK equivalence.
- `test_api.py` — every endpoint through the in-process TestClient.
