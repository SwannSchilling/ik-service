"""API tests: FastAPI endpoints over the C++ binding (in-process TestClient)."""
from fastapi.testclient import TestClient

from service.app import app

client = TestClient(app)


def test_health_and_solvers():
    r = client.get("/health")
    assert r.status_code == 200
    assert r.json()["status"] == "ok"
    assert r.json()["solvers"] == ["ccd", "gradient", "memetic"]
    assert client.get("/solvers").json()["solvers"] == ["ccd", "gradient", "memetic"]


def test_fk_zero_pose_mm_readout():
    r = client.post("/fk", json={"q": [0.0] * 7})
    assert r.status_code == 200
    d = r.json()
    assert len(d["frames"]) == 8
    assert d["tool0"]["position_mm"] == [0.0, 0.0, 1266.0]


def test_fk_rejects_bad_q():
    assert client.post("/fk", json={"q": [0.0] * 6}).status_code == 422


def test_solve_ccd_position_only_moderate_target():
    r = client.post("/solve", json={
        "solver": "ccd",
        "target": {"position": [0.45, 0.25, 0.45]},
    })
    assert r.status_code == 200
    d = r.json()
    assert d["success"] is True
    assert d["position_error"] < 1e-3
    assert d["orientation_error"] == -1.0
    assert d["solver"] == "ccd"
    assert d["time_ms"] >= 0
    assert len(d["q"]) == 7


def test_solve_gradient_moderate_target():
    r = client.post("/solve", json={
        "solver": "gradient",
        "target": {"position": [0.45, 0.25, 0.45]},
    })
    assert r.status_code == 200
    assert r.json()["success"] is True


def test_solve_memetic_deep_fold_target():
    r = client.post("/solve", json={
        "solver": "memetic",
        "target": {"position": [0.30, 0.20, 0.45]},
    })
    assert r.status_code == 200
    assert r.json()["success"] is True


def test_solve_full_pose_with_quaternion():
    r = client.post("/solve", json={
        "solver": "memetic",
        "target": {"position": [0.45, 0.25, 0.45], "quaternion": [0.0, 0.0, 0.0, 1.0]},
    })
    assert r.status_code == 200
    d = r.json()
    assert d["success"] is True
    assert d["orientation_error"] >= 0.0
    assert d["orientation_error"] < 1e-3


def test_solve_rounded_quaternion_is_normalized():
    """Regression: users type rounded quaternions (0.7071 for 1/sqrt(2)).
    Without normalization the target rotation matrix is slightly
    non-orthogonal (~0.006 rad off), so no true tool rotation can meet
    the 1e-3 rad orientation threshold and every full-pose goal failed.
    The service must normalize the quaternion into a proper rotation."""
    r = client.post("/solve", json={
        "solver": "gradient",
        "target": {
            "position": [0.45, 0.25, 0.45],
            "quaternion": [0.0, 0.0, 0.7071, 0.7071],  # 90 deg yaw, 4 decimals
        },
        "options": {"max_time": 2.0, "max_iterations": 2000},
    })
    assert r.status_code == 200
    d = r.json()
    assert d["success"] is True
    assert d["orientation_error"] < 1e-3
    assert d["position_error"] < 1e-3


def test_solve_zero_quaternion_rejected():
    r = client.post("/solve", json={
        "solver": "gradient",
        "target": {"position": [0.45, 0.25, 0.45], "quaternion": [0.0, 0.0, 0.0, 0.0]},
    })
    assert r.status_code == 422


def test_solve_minimal_displacement_weight():
    """The secondary seed-anchoring objective is forwarded to the binding
    (upstream PickIK's minimal_displacement_weight)."""
    r = client.post("/solve", json={
        "solver": "gradient",
        "target": {"position": [0.45, 0.25, 0.45]},
        "options": {
            "minimal_displacement_weight": 0.01,
            "cost_threshold": 0.05,  # goal check needs room (see solver.hpp)
            "max_time": 2.0,
            "max_iterations": 2000,
        },
    })
    assert r.status_code == 200
    d = r.json()
    assert d["success"] is True
    assert d["position_error"] < 5e-3
    assert len(d["q"]) == 7


def test_solve_memetic_options_forwarded():
    """elite_size / num_threads / population_size reach the solver."""
    r = client.post("/solve", json={
        "solver": "memetic",
        "target": {"position": [0.45, 0.25, 0.45]},
        "options": {
            "elite_size": 2,
            "population_size": 8,
            "num_threads": 2,
            "max_time": 1.0,
        },
    })
    assert r.status_code == 200
    assert r.json()["success"] is True


def test_solve_custom_seed():
    r = client.post("/solve", json={
        "solver": "ccd",
        "seed": [0.2, -0.4, 0.1, -0.8, 0.1, 0.3, -0.2],
        "target": {"position": [0.35, 0.20, 0.60]},
        "options": {"max_passes": 300},
    })
    assert r.status_code == 200
    assert r.json()["success"] is True


def test_solve_unknown_solver():
    r = client.post("/solve", json={
        "solver": "nope", "target": {"position": [0.45, 0.25, 0.45]},
    })
    assert r.status_code == 400


def test_solve_bad_quaternion():
    r = client.post("/solve", json={
        "solver": "ccd",
        "target": {"position": [0.45, 0.25, 0.45], "quaternion": [0.0, 0.0, 1.0]},
    })
    assert r.status_code == 422


def test_index_serves_demo():
    r = client.get("/")
    assert r.status_code == 200
    assert "pick_ik service" in r.text
