"""Tests for the SE(3) utilities."""

from __future__ import annotations

import numpy as np

from tio_former.utils.se3 import (
    axis_angle_to_quat,
    integrate_increment,
    matrix_to_quat,
    pose_to_matrix,
    quat_rotate,
    quat_to_axis_angle,
    quat_to_matrix,
    relative_pose,
    so3_exp,
    so3_log,
)


def test_quaternion_rotation() -> None:
    q = axis_angle_to_quat(np.array([0.0, 0.0, np.pi / 2.0]))
    rotated = quat_rotate(q, np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(rotated, [0.0, 1.0, 0.0], atol=1e-12)


def test_matrix_quaternion_roundtrip() -> None:
    q = axis_angle_to_quat(np.array([0.2, -0.5, 0.3]))
    q2 = matrix_to_quat(quat_to_matrix(q))
    np.testing.assert_allclose(np.abs(np.dot(q, q2)), 1.0, atol=1e-12)
    np.testing.assert_allclose(quat_to_matrix(q), quat_to_matrix(q2), atol=1e-12)


def test_so3_log_exp_roundtrip() -> None:
    vector = np.array([0.1, -0.2, 0.3])
    np.testing.assert_allclose(so3_log(so3_exp(vector)), vector, atol=1e-12)
    np.testing.assert_allclose(quat_to_axis_angle(axis_angle_to_quat(vector)), vector, atol=1e-12)


def test_relative_pose_is_body_frame() -> None:
    q = axis_angle_to_quat(np.array([0.0, 0.0, np.pi / 2.0]))
    previous = pose_to_matrix(np.array([1.0, 0.0, 0.0]), q)
    current = pose_to_matrix(np.array([1.0, 1.0, 0.0]), q)
    delta = relative_pose(previous, current)
    np.testing.assert_allclose(delta[:3, 3], [1.0, 0.0, 0.0], atol=1e-12)


def test_integrate_increment_matches_relative_pose() -> None:
    q = axis_angle_to_quat(np.array([0.0, 0.0, 0.4]))
    previous = pose_to_matrix(np.array([1.0, 2.0, 3.0]), q)
    dp = np.array([0.2, -0.1, 0.05])
    dphi = np.array([0.01, 0.02, -0.03])
    current = integrate_increment(previous, dp, dphi)
    recovered = relative_pose(previous, current)
    np.testing.assert_allclose(recovered[:3, 3], dp, atol=1e-12)
    np.testing.assert_allclose(so3_log(recovered[:3, :3]), dphi, atol=1e-12)
