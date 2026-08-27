"""FastAPI service over the `pickik` binding — transport layer only.

All IK runs in C++ (libpick_ik_core). This app does three things:
  1. exposes the solver contract over HTTP (POST /solve, POST /fk),
  2. owns the arm7 model (Python FK callback + robot limits, see arm7.py),
  3. serves the web demo (GET /).

Run:  python -m service.main     (from the ik_service directory)
Docs: http://127.0.0.1:8081/docs
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

import pickik
from . import arm7

app = FastAPI(title="pick_ik service", version="0.1.0")

# Dev/test transport layer: allow cross-origin calls from the p5.js POC
# sketch (which may run from file:// or a different local port) and from
# the web demo on this same origin alike.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

SOLVERS = ("ccd", "gradient", "memetic")
WEB_DIR = Path(__file__).resolve().parent.parent / "web"


# ---------------------------------------------------------------------------
# Request / response models
# ---------------------------------------------------------------------------

class TargetIn(BaseModel):
    position: list[float] = Field(..., description="Tool target [x, y, z] in meters.")
    quaternion: list[float] | None = Field(
        None,
        description="Optional orientation [x, y, z, w] (ROS/p5 order). "
                    "Omit for a position-only goal.",
    )


class FKIn(BaseModel):
    q: list[float] = Field(..., description="Joint positions [J1..J7] in radians.")


class SolveIn(BaseModel):
    solver: str = Field("memetic", description=f"One of {list(SOLVERS)}.")
    seed: list[float] | None = Field(
        None,
        description="Start configuration [J1..J7] rad. "
                    "Default: the POC's quantized 'all zero' slider state.",
    )
    target: TargetIn
    options: dict = Field(
        default_factory=dict,
        description="Optional: position_threshold, orientation_threshold, "
                    "cost_threshold, position_scale, rotation_scale, "
                    "minimal_displacement_weight, joint_angle_targets "
                    "(7 values, null = off) + joint_target_weight, look_at "
                    "{'point': [x,y,z] m, 'axis': [x,y,z]} + look_at_weight; "
                    "plus every solver-specific option: ccd "
                    "max_passes/damping/epsilon; gradient "
                    "step_size/min_cost_delta/max_time/max_iterations/"
                    "stop_optimization_on_valid_solution; memetic elite_size/"
                    "population_size/wipeout_fitness_tol/max_generations/"
                    "max_time/num_threads/stop_optimization_on_valid_solution/"
                    "stop_on_first_soln.",
    )


class SolveOut(BaseModel):
    success: bool
    q: list[float]
    position_error: float = Field(..., description="meters")
    orientation_error: float = Field(..., description="radians; -1 if not evaluated")
    solver: str
    time_ms: float


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_solver(name: str, options: dict) -> pickik.IkSolver:
    # Every solver option of the `pickik` binding is forwarded; the
    # defaults here intentionally differ from the binding's where they
    # matter for a service (wider budgets; single memetic species because
    # a Python FK serializes on the GIL pump — see the Phase-0 sweep).
    if name == "ccd":
        return pickik.CcdSolver(
            max_passes=int(options.get("max_passes", 600)),
            damping=float(options.get("damping", 0.1)),
            epsilon=float(options.get("epsilon", 1e-8)),
        )
    if name == "gradient":
        return pickik.PickIkGradientSolver(
            step_size=float(options.get("step_size", 0.0001)),
            min_cost_delta=float(options.get("min_cost_delta", 1e-12)),
            max_time=float(options.get("max_time", 2.0)),
            max_iterations=int(options.get("max_iterations", 2000)),
            stop_optimization_on_valid_solution=bool(
                options.get("stop_optimization_on_valid_solution", True)),
        )
    if name == "memetic":
        return pickik.PickIkMemeticSolver(
            elite_size=int(options.get("elite_size", 4)),
            population_size=int(options.get("population_size", 16)),
            wipeout_fitness_tol=float(options.get("wipeout_fitness_tol", 1e-5)),
            max_generations=int(options.get("max_generations", 100)),
            max_time=float(options.get("max_time", 2.0)),
            # 1 by default: every FK of every species goes through the GIL
            # pump on one Python thread, so extra species only add FK
            # traffic (measured: nt=1/elite~2 beats nt=4 across the board,
            # e.g. target B 55 ms vs 360 ms). Native FK hosts can raise it.
            num_threads=int(options.get("num_threads", 1)),
            stop_optimization_on_valid_solution=bool(
                options.get("stop_optimization_on_valid_solution", True)),
            stop_on_first_soln=bool(options.get("stop_on_first_soln", True)),
        )
    raise HTTPException(status_code=400, detail=f"unknown solver '{name}'")


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix from a quaternion [x, y, z, w] (no numpy.quaternion:
    it was removed in NumPy 2 and is not importable from older versions)."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]
    )


def _target_to_pose(target: TargetIn) -> np.ndarray:
    if len(target.position) != 3:
        raise HTTPException(status_code=422, detail="target.position must be [x, y, z]")
    pose = np.eye(4)
    pose[:3, 3] = target.position
    if target.quaternion is not None:
        if len(target.quaternion) != 4:
            raise HTTPException(
                status_code=422, detail="target.quaternion must be [x, y, z, w]"
            )
        q = np.asarray(target.quaternion, dtype=float)
        # Normalize: callers send rounded quaternions (e.g. 0.7071 for
        # 1/sqrt(2)). An unnormalized quaternion yields a slightly
        # non-orthogonal rotation matrix (~0.006 rad off for 4-decimal
        # input) — unreachable under the default 1e-3 rad orientation
        # threshold, which made every full-pose goal "unsolvable".
        norm = float(np.linalg.norm(q))
        if norm < 1e-12:
            raise HTTPException(
                status_code=422, detail="target.quaternion must be non-zero"
            )
        pose[:3, :3] = _quat_to_matrix(*(q / norm).tolist())
    return pose


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@app.get("/solvers")
def solvers() -> dict:
    return {"solvers": list(SOLVERS)}


@app.get("/health")
def health() -> dict:
    return {
        "status": "ok",
        "solvers": list(SOLVERS),
        "arm": "arm7 (POC 7-DOF, meters/radians)",
        "fk": "Python reference (service/arm7.py)",
    }


@app.post("/fk")
def fk(req: FKIn) -> dict:
    """Forward kinematics: {q: [J1..J7] rad} -> joint frames + tool0."""
    q = req.q
    try:
        frames = arm7.link_frames(q)
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))
    tool0 = frames[-1]
    return {
        "frames": [f.tolist() for f in frames],
        "tool0": {
            "position_m": tool0[:3, 3].tolist(),
            "position_mm": (tool0[:3, 3] * 1000.0).tolist(),
        },
    }


@app.post("/solve", response_model=SolveOut)
def solve(req: SolveIn) -> SolveOut:
    """Run one IK solve through the C++ solver contract."""
    if req.solver not in SOLVERS:
        raise HTTPException(status_code=400, detail=f"unknown solver '{req.solver}'")

    pose = _target_to_pose(req.target)
    position_only = req.target.quaternion is None

    options = pickik.SolveOptions()
    options.position_threshold = float(req.options.get("position_threshold", 1e-3))
    options.cost_threshold = float(req.options.get("cost_threshold", 1e-3))
    options.position_scale = float(req.options.get("position_scale", 1.0))
    options.minimal_displacement_weight = float(
        req.options.get("minimal_displacement_weight", 0.0)
    )

    # Per-joint angle targets: [null|number x 7]; null = no target.
    jats = req.options.get("joint_angle_targets")
    if jats is not None:
        if not isinstance(jats, (list, tuple)) or len(jats) != 7:
            raise HTTPException(
                status_code=422, detail="joint_angle_targets must be 7 values (null or rad)"
            )
        options.joint_angle_targets = [
            None if v is None else float(v) for v in jats
        ]
        options.joint_target_weight = float(
            req.options.get("joint_target_weight", 0.0)
        )

    # Look-at target: {"point": [x, y, z] m, "axis": [x, y, z] (default +X)}.
    la = req.options.get("look_at")
    if la is not None:
        if not isinstance(la, dict) or "point" not in la:
            raise HTTPException(
                status_code=422, detail="look_at must be {'point': [x, y, z] m}"
            )
        point = la["point"]
        if not isinstance(point, (list, tuple)) or len(point) != 3:
            raise HTTPException(
                status_code=422, detail="look_at.point must be [x, y, z] meters"
            )
        axis = la.get("axis", [1.0, 0.0, 0.0])
        if not isinstance(axis, (list, tuple)) or len(axis) != 3:
            raise HTTPException(
                status_code=422, detail="look_at.axis must be [x, y, z]"
            )
        options.look_at = pickik.LookAtTarget(
            [float(v) for v in point], [float(v) for v in axis]
        )
        options.look_at_weight = float(req.options.get("look_at_weight", 0.0))

    if position_only:
        options.orientation_threshold = None
        options.rotation_scale = float(req.options.get("rotation_scale", 0.0))
    else:
        options.orientation_threshold = float(
            req.options.get("orientation_threshold", 1e-3)
        )
        options.rotation_scale = float(req.options.get("rotation_scale", 0.5))

    seed = req.seed if req.seed is not None else list(arm7.QUANTIZED_ZERO_SEED)
    if len(seed) != 7:
        raise HTTPException(status_code=422, detail="seed must have 7 values")

    solver = _make_solver(req.solver, req.options)
    t0 = time.perf_counter()
    try:
        result = solver.solve(
            arm7.ROBOT, arm7.fk_callback, arm7.LOCAL_AXES, seed, [pose], options
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    return SolveOut(
        success=result.success,
        q=result.q,
        position_error=result.position_error,
        orientation_error=result.orientation_error,
        solver=req.solver,
        time_ms=elapsed_ms,
    )


@app.get("/")
def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")
