"""Repro for the intermittent memetic crash.

Mimics the FastAPI service: solve is invoked from NON-main worker threads,
and multiple solves can run concurrently (uvicorn/anyio threadpool). Each
memetic solve itself fans out to num_threads impl-threads, each of which
spawns eliteCount() gradientDescent threads, all of which call the Python FK
callback (GIL-serialized).

Run:  python repro_memetic_storm.py
"""
from __future__ import annotations

import faulthandler
import sys
import threading
import traceback
from pathlib import Path

faulthandler.enable(all_threads=True)

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import _bootstrap  # noqa: E402  (vendor + dist)

import ctypes  # noqa: E402
import numpy as np  # noqa: E402
import pickik       # noqa: E402
from service import arm7  # noqa: E402

# Native crash forensics: VEH writes module+offset stacks on any AV.
_CRASH_DUMP = str(Path(__file__).resolve().parent.parent / ".tmp" / "veh_dump" / "crash_stack.txt")
try:
    if Path(_CRASH_DUMP).exists():
        Path(_CRASH_DUMP).unlink()
    _veh = ctypes.CDLL(str(Path(__file__).resolve().parent.parent / ".tmp" / "veh_dump" / "veh_dump.dll"))
    _veh.InstallVehCrashDump(_CRASH_DUMP)
except OSError as e:
    print(f"(VEH not installed: {e})", flush=True)

import os  # noqa: E402

TARGET = [0.45, 0.25, 0.45]
# Bisection knobs: N_WORKERS = concurrent top-level solves (like parallel
# HTTP requests), IMPL_THREADS = memetic's num_threads (C++ species threads).
N_WORKERS = int(os.environ.get("STORM_WORKERS", "4"))
ROUNDS = int(os.environ.get("STORM_ROUNDS", "15"))
IMPL_THREADS = int(os.environ.get("STORM_THREADS", "4"))

errors: list[str] = []
lock = threading.Lock()
done = threading.Event()


def worker(idx: int) -> None:
    seed = list(arm7.QUANTIZED_ZERO_SEED)
    for r in range(ROUNDS):
        try:
            solver = pickik.PickIkMemeticSolver(num_threads=IMPL_THREADS, max_time=0.6)
            opts = pickik.SolveOptions()
            opts.orientation_threshold = None
            opts.rotation_scale = 0.0
            pose = np.eye(4)
            pose[:3, 3] = TARGET
            res = solver.solve(arm7.ROBOT, arm7.fk_callback, arm7.LOCAL_AXES,
                               seed, [pose], opts)
            with lock:
                print(f"[w{idx}] round {r:02d} ok success={res.success} "
                      f"pos_err={res.position_error*1000:.4f}mm", flush=True)
        except BaseException:
            with lock:
                errors.append(f"[w{idx}] round {r}: " + traceback.format_exc())
                print(f"[w{idx}] round {r} EXCEPTION:\n" + traceback.format_exc(),
                      flush=True)


def main() -> None:
    print(f"Starting {N_WORKERS} workers x {ROUNDS} rounds, "
          f"memetic num_threads={IMPL_THREADS}", flush=True)
    threads = [threading.Thread(target=worker, args=(i,), name=f"solve-w{i}")
               for i in range(N_WORKERS)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    done.set()
    print(f"\nDone. {len(errors)} exceptions.")
    if errors:
        print("First error:\n" + errors[0])
        sys.exit(1)


if __name__ == "__main__":
    main()
