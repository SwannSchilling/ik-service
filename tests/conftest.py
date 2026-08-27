"""Path setup: make `service` (package) and `pickik` (C++ binding, dist/) importable."""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, os.path.join(ROOT, "dist")):
    if p not in sys.path:
        sys.path.insert(0, p)

import _bootstrap  # noqa: E402  (shadows system Starlette 1.x with vendored 0.41.2)
