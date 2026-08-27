# Vendored dependencies

## starlette 0.41.2

- **Provenance**: copied from a locally maintained venv
  (`Documents/Code/LanthanToolbox/venv`, starlette-0.41.2.dist-info);
  upstream: https://github.com/encode/starlette (MIT License,
  https://github.com/encode/starlette/blob/0.41.2/LICENCE — the vendored
  package copy does not ship the license file; see upstream).
- **Why**: this environment cannot reach PyPI, and the system site-packages
  carries Starlette 1.3.1, which breaks FastAPI 0.115.4
  (`Router.__init__() got an unexpected keyword argument 'on_startup'`).
  FastAPI 0.115.x supports Starlette >=0.40,<0.42; 0.41.2 satisfies that.
- **How**: `ik_service/_bootstrap.py` puts this directory first on
  `sys.path` (import it before fastapi/uvicorn — `service/main.py` and
  `tests/conftest.py` already do). The vendored copy shadows the system
  Starlette for anything that runs through those entry points.
- **Removing it**: `pip install 'starlette<0.42'` (when PyPI is reachable)
  and drop this directory plus the `_bootstrap` imports.
