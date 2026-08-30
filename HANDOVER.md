# HANDOVER — read this first (living document)

> Machine-agnostic resume for this project. **Any agent or developer starts a
> session by reading this file and updates it before pushing.** A stale
> handover is worse than none — keep "Current state" / "Next up" truthful.
>
> Last updated: 2026-08-28 — **desktop-arm Design B (675 mm) implemented:**
> all ports rescaled and verified (spec anchors regenerated; ctest 26/26;
> cross-check + pytest 44/44; URDF visuals rebuilt at the new dimensions;
> demo sliders/camera rescaled; new cross-check targets 200/100/300 and
> 300/150/300 mm) — see §4; earlier: second-machine onboarding (both builds
> green, two CMake build fixes), URDF-driven 3D viewer (applyMatrix fix
> `cebb2ae`), Windows launcher scripts (`start_service.bat` /
> `stop_service.bat`); the rescale study (`libpick-ik-core/docs/
> desktop-arm-design-study.md`) is closed at "implemented" with its §11
> checklist annotated; C ABI + shared arm7 header (§3.0a/b/c) landed in
> libpick-ik-core (c40f47a, 9931105); the Blender 4.x add-on (§3.1 v1) is
> done and passes its 5-gate headless acceptance on Blender 4.5.3 —
> including the static-CRT plugin-ABI fix for Blender's pinned MSVCP140
> (98b000a); remaining: add-on repo hosting decision + §3.2 Unity.

## 1. Project in one paragraph

A 7-DOF URDF arm ("arm7") plus an IK pipeline. The Pick IK solvers
(PickNikRobotics/pick_ik, BSD-3-Clause) are extracted into a standalone,
ROS/MoveIt-free C++ core (`libpick_ik_core`); a pybind11 binding (`pickik`)
exposes an `IkSolver` contract (CCD / gradient / memetic); this repo is a
FastAPI service on top with `/solve`, `/fk`, `/solvers` and a p5.js 3D web
demo that renders the arm from the URDF + STL meshes in `robot_description/`.
Next work item: Unity native (§3.2), reusing the `pick_ik_c` C ABI (done,
`libpick_ik_core` §3.0a/b/c: `pickik_c.h` + `pickik_c.cpp`, shared
`examples/arm7/arm7.hpp`, C-ABI ctest suite; `pick_ik_c` builds with the
static MSVC CRT so hosts pinning their own older C++ runtime — Blender's
`blender.crt/msvcp140.dll` 14.29 — cannot break its thread primitives).
The Blender add-on (§3.1 v1) lives in `blender_ik_addon/` at the workspace
root (folder not yet a repo — hosting decision pending): ctypes over
`pick_ik_c.dll`, empty-object rig, target gizmo, solver dropdown, Solve +
continuous mode; 5/5 acceptance gates on Blender 4.5.3 headless.

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
- [x] **Rescale arm7 to a desktop arm — Design B implemented (2026-08).**
      The iiwa-class POC dimensions (340/400/400 mm + tool 126 mm →
      1.266 m) were replaced by **Design B (base→J2 180, J2→J4 215,
      J4→J6 215, J6→tool 65 mm = 675 mm chain)** per the study
      (`libpick-ik-core/docs/desktop-arm-design-study.md`, status:
      implemented, §11 checklist annotated). J2 = CubeMars AK10-9 V2.0 KV60
      (18/53 Nm, Ø98×61.7), J4 = AK70-10 (8.3/24.8 Nm, Ø89×50.25 — revision
      to verify); J1/J3/J5/J6/J7 and structural masses remain open
      (study §10). Joint convention, limits (±π / ±2.09), velocities and
      axes unchanged — only the four linear constants moved, in every port:
      spec §5 anchors regenerated (zero (0,0,0.675), shoulder fwd
      (0.495,0,0.180), elbow fwd (0.280,0,0.395), wrist fwd
      (0.065,0,0.610)); C++ test ports + `arm7_cross_check` rebuilt
      (ctest 26/26; new cross-check targets **200/100/300** (deep fold —
      CCD pins J6 at the 2.09 limit) and **300/150/300 mm** (moderate, no
      pinned joints) solved by all three solvers); `service/arm7.py` +
      pytest (44/44, anchors + targets + service smoke: /fk zero pose
      675 mm, /solve both targets, clean no-solve at the ≈0.151 m inner
      boundary); URDF visuals rebuilt at the new dimensions (50–60 mm
      structural links, motor-sized pivot spheres, effort attrs 18 / 8.3
      Nm); demo sliders x/y −550…550, z 0…650, defaults 300/150/300, camera
      1.8 m (clamp 0.3–3.0), grid ±0.6 m. Fold degeneracy (study §6.1):
      d(θ4) = 2·0.215·cos(θ4/2) stays ≥ 0.216 m under the ±2.09 limits, so
      the degenerate fold is out of the limit box — open item: soft
      penalty for d < 0.300 m.

In progress / next up:
- [x] **Blender 4.x add-on (v1) — done 2026-08-30.** `blender_ik_addon/` at
      the workspace root: ctypes over `pick_ik_c.dll`, empty-object rig +
      target gizmo, solver dropdown (gradient default / ccd / memetic),
      Solve + 50 ms continuous timer, md weight, status line. 5/5 acceptance
      gates headless on Blender 4.5.3 (anchors, target B gradient 0.68 mm,
      target A memetic-on-background-thread 0.8 µm, out-of-workspace,
      stall p90 ~3.8 ms).
      **Open decision (blocks nothing):** where the add-on code lives —
      suggested: its own small repo (`blender_ik_addon`); no `gh` CLI on
      this machine, so the repo (if any) must be created from the GitHub
      web. v1.1 backlog: per-joint targets + look-at in the panel (C ABI
      already plumbs both), optional STL display.
- [ ] Unity native (§3.2) and PyBullet/IsaacSim (§3.3) — next; the C ABI
      (now static-CRT) is the entry point.

## 5. Expensive lessons (read before touching rendering, model, threading)

- **p5 v2.3.2 `applyMatrix` reads its 16 arguments in column-major order.**
  FK frames from `/fk` are row-major, so the 3×3 block must be passed
  transposed (column-major). Empirically verified; fixed in `cebb2ae`. Do not
  "fix" it back. Full probe matrix:
  `docs/session-logs/session-log-3d-viewer-urdf-fix.md`.
- **p5's world is pixel-scale** (demo uses SCALE = 300 px/m); camera
  up-vector is `(0,0,-1)`; `stroke()` renders nothing on closed 3D surfaces.
- **The arm7 model is triple-ported** (C++ `tests/arm7_fk.hpp` /
  `examples/arm7_cross_check`, Python `service/arm7.py`, plus the URDF in
  `robot_description/` which must mirror the chain; the historical JS port
  is the machine-local p5 POC, superseded by `web/index.html`).
  `libpick_ik_core/docs/arm7-kinematic-spec.md` is the single source of truth
  (CAD source of truth, "MATH CONFIRMED"); pytest pins `arm7.py` to the
  spec's §5 anchor poses. Model change → C++ table first, re-run
  `arm7_cross_check` + ctest, then propagate (spec §7).
  Known solver quirk (found during the Design B rescale): the memetic's
  random population seeding can stall at the seed on full-pose goals with
  large orientation deltas (upstream behavior — use gradient for those);
  position-only targets are unaffected.
- **URDF loading is cosmetic; the solver model is hardcoded in code.**
  `robot_description/*.urdf` (+ the STLs) is consumed only by the web
  viewer — it places meshes on the FK frames. The model the solver actually
  solves against (joint axes/offsets, link geometry via the FK callback,
  joint limits) comes from `service/arm7.py` (`JOINTS` + `TOOL_OFFSET` →
  `pickik.make_robot`), ported 1:1 from the spec; the `pickik` binding
  itself is model-agnostic. Loading a different URDF renders that model
  but poses it as arm7. A generic / URDF-driven solver is a future item;
  the roadmap §3.0b model header is the first step toward
  model/solver decoupling.
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
