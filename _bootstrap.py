"""Import-time bootstrap for ik_service.

Two jobs:
  1. Starlette version shadowing: the system Python has Starlette 1.x
     installed, which is incompatible with the installed FastAPI 0.115.x
     (FastAPI passes on_startup/on_shutdown to starlette's Router, which
     Starlette 1.x removed). This workspace vendors Starlette 0.41.2 (MIT,
     see vendor/README.md) and puts it first on sys.path so it shadows the
     system copy.
  2. The `pickik` C++ binding (built into ./dist/) is made importable.

Import this module before importing fastapi/starlette/uvicorn:

    import _bootstrap
"""
from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent


def ensure() -> None:
    for p in (_ROOT / "dist", _ROOT / "vendor"):  # vendor ends up first
        if p.is_dir() and str(p) not in sys.path:
            sys.path.insert(0, str(p))


ensure()
