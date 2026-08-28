# HANDOVER — read this first (living document)

> Machine-agnostic resume for this project. **Any agent or developer starts a
> session by reading this file and updates it before pushing.** A stale
> handover is worse than none — keep "Current state" / "Next up" truthful.
>
> Last updated: 2026-08-28 — second-machine onboarding done (both builds
> green, 26 ctest + 44 pytest; two CMake build fixes); URDF-driven 3D viewer
> done (applyMatrix fix `cebb2ae`); Windows launcher scripts added
> (`start_service.bat` / `stop_service.bat`); Blender add-on is the next item.

## 1. Project in one paragraph

A 7-DOF URDF arm ("arm7") plus an IK pipeline. The Pick IK solvers
(PickNikRobotics/pick_ik, BSD-3-Clause) are extracted into a standalone,
ROS/MoveIt-free C++ core (`libpick_ik_core`); a pybind11 binding (`pickik`)
exposes an `IkSolver` contract (CCD / gradient / memetic); this repo is a
FastAPI service on top with `/solve`, `/fk`, `/solvers` and a p5.js 3D web
demo that renders the arm from the URDF + STL meshes in `robot_description/`.
Next work item: the Blender 4.x add-on (`libpick_ik_core/docs/
integration-roadmap.md` §3.1), preceded by the `pick_ik_c` C ABI layer
(§3.0a) and the shared C++ arm7 model header (§3.0b).

## 2. Repos and layout

| Repo | Content |
|---|---|
| `SwannSchilling/libpick-ik-core` | C++ core, `pickik` binding, tests, docs (api-reference, integration-roadmap, arm7 spec, extraction analysis) |
| `SwannSchilling/ik-service` (this repo) | FastAPI service, p5.js web demo, pytest, `robot_description/` (URDF + STL, served via `GET /model/*`) |

- **Both must be sibling folders.** `ik_service/CMakeLists.txt` configures
  `../libpick_ik_core` and builds `pickik` into `./dist/`.
- `../.deps/` (machine-local, **not** in any repo): shallow clones of
  Eigen/fmt/Catch2/pybind11 used as `FETCHCONTENT_SOURCE_DIR_*` overrides on
  machines with restricted network. Without them CMake clones from the
  network at configure time.
- A read-only reference clone of upstream `PickNikRobotics/pick_ik` may exist
  on dev machines (`pick_ik/`) — never modify or push it.
- The original p5 POC folder (`RobotArm_2026_08_25_10_03_56`) is machine-local
  and superseded by `web/index.html` — intentionally not in any repo.

## 3. Build / run / test (dev machine)

Prereqs: Python 3.10+ (dev machine: MS-Store CPython 3.12.28), CMake ≥ 3.22,
MSVC via Visual Studio 2022.

```sh
# core (from libpick_ik_core/):
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config RelWithDebInfo
ctest --test-dir build -C RelWithDebInfo

# service + binding (from ik_service/):
pip install -r requirements.txt
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 \
  -DFETCHCONTENT_SOURCE_DIR_EIGEN=../.deps/eigen \
  -DFETCHCONTENT_SOURCE_DIR_FMT=../.deps/fmt \
  -DFETCHCONTENT_SOURCE_DIR_PYBIND11=../.deps/pybind11
cmake --build build --config RelWithDebInfo    # -> dist/pickik.pyd
pytest tests/ -q -p no:cacheprovider           # 44 tests
python -m service.main                         # http://127.0.0.1:8081/
```

Windows launcher helpers (repo root, CRLF, self-contained): `start_service.bat`
starts the service in its own "ik_service" console window, polls `/health`
and opens the web demo; if an instance already owns the port it reports the
PID and only opens the browser, and `start_service.bat /restart` kills the
running instance first (refuses to kill any PID that is not `python.exe`)
before starting fresh. `stop_service.bat` stops the running instance
(graceful `taskkill /PID`, then `taskkill /F` after 5 s, then verifies the
port is free). Port discovery uses PowerShell `Get-NetTCPConnection` because
`netstat` state names are localized on non-English systems; in-window sleeps
use loopback `ping`, since `timeout.exe` aborts under redirected stdin.

## 4. Current state

Done:
- [x] `libpick_ik_core` extraction (ik_gradient/ik_memetic byte-identical to
      upstream; pure robot/goal + `Robot::make`) — provenance in core README
- [x] `IkSolver` contract + `CcdSolver` (C++ port of the POC's CCD) +
      gradient/memetic wrappers
- [x] `pickik` pybind11 binding with the FK-pump threading design (two crash
      post-mortems in the core README)
- [x] `ik_service`: FastAPI `/solve`, `/fk`, `/solvers`, `/health` + p5.js
      demo; 44 pytest tests
- [x] URDF-driven 3D viewer: `robot_description/` folder, `GET /model/*`
      endpoint, STL meshes; applyMatrix column-major fix (`cebb2ae`)
- [x] Both repos public on GitHub (`git@github.com:SwannSchilling/...`),
      pushed via SSH
- [x] Second-machine onboarding (swann_gmt71y box, 2026-08-28): full build
      + test green on Python 3.13.5 / CMake 3.27.0-rc3; two build fixes
      (core install interface, service sibling-folder spelling) — see
      §7 machine notes
- [x] Windows launcher scripts `start_service.bat` / `stop_service.bat`
      (start / stop / `/restart`, browser open, PID-verified kills)

In progress / next up:
- [ ] **Blender 4.x add-on** (roadmap §3.1). Prerequisites: `pick_ik_c` C ABI
      layer (§3.0a — thin C interface over the `IkSolver` contract, no new
      solver code) and the shared C++ arm7 model header (§3.0b,
      `examples/arm7/arm7.hpp`). Shape: add-on loads `pick_ik_c` via ctypes,
      target gizmo, solver dropdown, Solve button + optional continuous mode.
      Acceptance: §3.1 (spec anchors + targets A/B reproduced; no main-thread
      stall > 4 ms) via the §3.0c validation protocol.
      **Open decision:** where the add-on code lives — suggested: its own
      small repo (`blender_ik_addon`) to keep Blender packaging separate from
      the service.
- [ ] Unity native (§3.2) and PyBullet/IsaacSim (§3.3) — after Blender.

## 5. Expensive lessons (read before touching rendering, model, threading)

- **p5 v2.3.2 `applyMatrix` reads its 16 arguments in column-major order.**
  FK frames from `/fk` are row-major, so the 3×3 block must be passed
  transposed (column-major). Empirically verified; fixed in `cebb2ae`. Do not
  "fix" it back. Full probe matrix:
  `docs/session-logs/session-log-3d-viewer-urdf-fix.md`.
- **p5's world is pixel-scale** (demo uses SCALE = 300 px/m); camera
  up-vector is `(0,0,-1)`; `stroke()` renders nothing on closed 3D surfaces.
- **The arm7 model is triple-ported** (C++ `tests/arm7_fk.hpp` /
  `examples/arm7_cross_check`, Python `service/arm7.py`, JS in the p5 POC).
  `libpick_ik_core/docs/arm7-kinematic-spec.md` is the single source of truth
  (CAD source of truth, "MATH CONFIRMED"); pytest pins `arm7.py` to the
  spec's §5 anchor poses. Model change → update spec → update all ports →
  re-run the cross-checks.
- **Memetic + Python FK:** FK is evaluated on the calling thread through the
  GIL pump; extra `num_threads` only add queue traffic, no parallelism.
  Service default is `num_threads=1` (keep it); native hosts should raise it.
- **Starlette is vendored at 0.41.2** in `vendor/` — system Starlette 1.x
  breaks FastAPI 0.115. Don't "fix" the import.
- **Port 8081**, not 8000 (8000 is inside a Windows excluded TCP range on the
  dev machine; override with `IK_SERVICE_PORT`).
- **pytest needs `-p no:cacheprovider`** under the DSH sandbox (cache-file
  writes are denied).
- **`ik_service` contains no solver code** — all solving happens in the core.
  When behaviour looks wrong, look in `libpick_ik_core` first.

## 6. Cross-machine work protocol

Chat history does not travel between machines — **this file and git do**.
Machine A's agent never sees machine B's conversation. So:

**Session start (every machine):**
```sh
git -C libpick_ik_core pull
git -C ik_service pull
```
Read this file, then `libpick_ik_core/HANDOVER.md`, then run the test suites
(`ctest` + `pytest`) to confirm the repo state matches the handover claims.

**Session end (every machine):**
1. Update "Current state" / "Next up" above (one bullet per significant event).
2. Model change? Spec + anchor tests updated first.
3. Commit with a conventional prefix (`feat|fix|docs|test|chore`); write
   commit bodies that stand alone in `git log` — commits are a communication
   channel too.
4. Push.

## 7. Machine notes

**Second machine (swann_gmt71y Windows box, filled in on first use 2026-08-28):**
- Prereqs all present: Python 3.13.5 default (`C:\Python313`; requirements
  comment says 3.12 but 3.10+ works), CMake 3.27.0-rc3, VS 2022 Community
  (MSVC 14.42.34433), git 2.41 + OpenSSH 9.3, network to github/gitlab OK.
- **No `../.deps` cache** — FetchContent clones Eigen/fmt/Catch2/pybind11
  from the network at configure time (omit the `FETCHCONTENT_SOURCE_DIR_*`
  overrides from the commands above on this box).
- Repos are cloned with the GitHub names: `ik-service/`, `libpick-ik-core/`
  (hyphens, not the dev machine's underscore spelling). Service CMake now
  accepts both (`fix: accept libpick-ik-core sibling spelled with hyphen or
  underscore`).
- **Build the binding with `-DPYTHON_EXECUTABLE=C:/Python313/python.exe`.**
  pybind11 v2.13.6's legacy finder otherwise auto-detects an unrelated
  portable CPython 3.12.13 (`~/.local/bin/python3.12.exe`, the Reachy Mini
  Control project's embedded build) and produces a `cp312` pyd that the
  default Python 3.13 cannot load.
- Remotes are SSH (`git@github.com:SwannSchilling/...`), pushed with the
  dedicated key `~/.ssh/id_ed25519_github_ik` (registered on the GitHub
  account, comment `ik-repos on swannbox`) — same pattern as the dev
  machine; the `github.com` block lives in `~/.ssh/config` and the private
  key is owner-only ACL'd. Verified 2026-08-28: both fix commits pushed.
  (Note: this key is account-level, like the dev machine's — it can push
  to every repo the account has write access to, not just the ik-repos.)
- Port 8081 is NOT in the Windows excluded TCP ranges here (the dev
  machine's port-8000 problem does not apply).
- Blender 4.5 installed (`C:\Program Files\Blender Foundation`) — ready
  for the add-on work (§3.1) once `pick_ik_c` lands.
- 2026-08-28 onboarding result: core builds, 26/26 ctest; service builds
  (cp313), 44/44 pytest; `/health`, `/solvers`, `/solve` (ccd) smoke-tested
  OK on 8081. Build fix in the core: RSL includes wrapped in
  BUILD_INTERFACE/INSTALL_INTERFACE — a raw source path in the install
  interface made CMake's `install(EXPORT)` generation fatal on any CMake
  ≥3.22 for out-of-source builds (`fix: keep RSL source include out of the
  install interface`).

**Machine of this agent (dev machine, LENOVO03):**
- DSH file sandbox: git's bundled Cygwin `ssh.exe` cannot create named pipes,
  so under the sandbox set
  `$env:GIT_SSH = "C:\Windows\System32\OpenSSH\ssh.exe"` before any
  `git push` / `git ls-remote`. On a normal terminal this is not needed.
- SSH: `~/.ssh/id_ed25519_github` (public key registered on the GitHub
  account, comment `ik-repos on LENOVO03`); `~/.ssh/config` routes
  `git@github.com` to it.
- `../.deps` shallow clones present; the service runs as a managed background
  job (`python -m service.main` from `ik_service/`).

**Second machine (fill in on first use):**
- prereqs installed? Python version? `.deps` present or network fetching?
  ssh key registered on the GitHub account?
