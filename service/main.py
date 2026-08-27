"""Run the pick_ik service locally.

Usage (from the ik_service directory):
    python -m service.main
Then:
    demo UI:  http://127.0.0.1:8081/
    OpenAPI:  http://127.0.0.1:8081/docs
"""
import _bootstrap  # must come before fastapi/uvicorn (shadows Starlette 1.x)

import os

import uvicorn

# Port 8000 is inside a Windows excluded TCP range on this machine
# (`netsh interface ipv4 show excludedportrange protocol=tcp`), so the
# service defaults to 8081. Override with IK_SERVICE_PORT if needed.
if __name__ == "__main__":
    uvicorn.run(
        "service.app:app",
        host="127.0.0.1",
        port=int(os.environ.get("IK_SERVICE_PORT", "8081")),
    )
