"""End-to-end tests of the pickik binding: Python FK + C++ solvers.

Targets are the Design B cross-check set:
  A "deep fold" 200/100/300 mm — every solution pins J4/J6 at the 2.09 limit
  B "moderate"   300/150/300 mm — reachable with no joint at its limit
"""
import numpy as np
import pytest

import pickik
from service import arm7

TARGET_A = np.eye(4)
TARGET_A[:3, 3] = [0.20, 0.10, 0.30]  # deep fold 200/100/300 mm
TARGET_B = np.eye(4)
TARGET_B[:3, 3] = [0.30, 0.15, 0.30]  # moderate 300/150/300 mm


def pos_only() -> pickik.SolveOptions:
    o = pickik.SolveOptions()
    o.orientation_threshold = None
    o.rotation_scale = 0.0
    return o


def test_robot_surface():
    assert arm7.ROBOT.num_joints == 7
    assert arm7.ROBOT.is_valid_configuration([0.0] * 7)
    assert not arm7.ROBOT.is_valid_configuration([4.0] + [0.0] * 6)  # J1 > pi
    assert not arm7.ROBOT.is_valid_configuration([0.0, 3.0] + [0.0] * 5)  # J2 > 2.09
    spec = pickik.JointSpec(-1.0, 1.0, max_velocity=2.5)
    assert spec.min == -1.0 and spec.max == 1.0 and spec.max_velocity == 2.5
    assert spec.bounded is True


def test_solver_names():
    assert pickik.CcdSolver().name() == "ccd"
    assert pickik.PickIkGradientSolver().name() == "gradient"
    assert pickik.PickIkMemeticSolver().name() == "memetic"


def test_ccd_solves_both_poc_targets():
    ccd = pickik.CcdSolver(max_passes=600)
    for target in (TARGET_A, TARGET_B):
        result = ccd.solve(arm7.ROBOT, arm7.fk_callback, arm7.LOCAL_AXES,
                           list(arm7.QUANTIZED_ZERO_SEED), [target], pos_only())
        assert result.success
        assert result.position_error < 1e-3
        assert result.orientation_error == -1.0  # position-only solver
        assert arm7.ROBOT.is_valid_configuration(result.q)
        # independent FK check: the returned q really is at the target
        np.testing.assert_allclose(
            arm7.tool0_pose(result.q)[:3, 3], target[:3, 3], atol=result.position_error)


def test_gradient_solves_moderate_target():
    gd = pickik.PickIkGradientSolver(max_time=2.0, max_iterations=2000)
    result = gd.solve(arm7.ROBOT, arm7.fk_callback, arm7.LOCAL_AXES,
                      list(arm7.QUANTIZED_ZERO_SEED), [TARGET_B], pos_only())
    assert result.success
    assert result.position_error < 1e-3
    assert arm7.ROBOT.is_valid_configuration(result.q)


def test_memetic_solves_deep_fold_target():
    me = pickik.PickIkMemeticSolver(num_threads=4, max_time=2.0)
    result = me.solve(arm7.ROBOT, arm7.fk_callback, arm7.LOCAL_AXES,
                      list(arm7.QUANTIZED_ZERO_SEED), [TARGET_A], pos_only())
    assert result.success
    assert result.position_error < 1e-3
    assert arm7.ROBOT.is_valid_configuration(result.q)


def test_full_pose_goal_evaluates_orientation():
    # Full pose (position + orientation). The goal is FK of a known
    # configuration (no joint near its limit), so it is reachable by
    # construction — the assertions are on the metric itself, not on a
    # hand-picked orientation. Uses the gradient solver (deterministic):
    # for large orientation deltas the memetic's random population
    # seeding measurably stalls at the zero seed from run to run
    # (upstream behavior; memetic full-pose reliability is covered by the
    # C++ ctest suite).
    gd = pickik.PickIkGradientSolver(max_time=2.0, max_iterations=2000)
    options = pickik.SolveOptions()  # full pose by default
    q_true = [0.30, -0.40, 0.10, -0.80, 0.15, 0.35, -0.20]
    goal = arm7.tool0_pose(q_true)
    result = gd.solve(arm7.ROBOT, arm7.fk_callback, arm7.LOCAL_AXES,
                      list(arm7.QUANTIZED_ZERO_SEED), [goal], options)
    assert result.success
    assert result.position_error < 1e-3
    assert result.orientation_error >= 0.0
    assert result.orientation_error < options.orientation_threshold


def test_seed_size_mismatch_raises():
    ccd = pickik.CcdSolver()
    with pytest.raises(ValueError):
        ccd.solve(arm7.ROBOT, arm7.fk_callback, arm7.LOCAL_AXES,
                  [0.0] * 6, [TARGET_B], pos_only())


def test_fk_returning_wrong_frame_count_raises():
    ccd = pickik.CcdSolver()

    def bad_fk(q):
        return arm7.link_frames(q)[:7]  # missing the tip frame

    with pytest.raises(ValueError):
        ccd.solve(arm7.ROBOT, bad_fk, arm7.LOCAL_AXES,
                  list(arm7.QUANTIZED_ZERO_SEED), [TARGET_B], pos_only())


def test_numpy_and_list_frames_are_equivalent():
    ccd = pickik.CcdSolver(max_passes=100)
    opts = pos_only()

    def fk_numpy(q):
        return arm7.link_frames(q)  # numpy arrays

    def fk_lists(q):
        return [f.tolist() for f in arm7.link_frames(q)]  # plain nested lists

    result_np = ccd.solve(arm7.ROBOT, fk_numpy, arm7.LOCAL_AXES,
                          list(arm7.QUANTIZED_ZERO_SEED), [TARGET_B], opts)
    result_lst = ccd.solve(arm7.ROBOT, fk_lists, arm7.LOCAL_AXES,
                           list(arm7.QUANTIZED_ZERO_SEED), [TARGET_B], opts)
    np.testing.assert_allclose(result_np.q, result_lst.q, atol=1e-15)
