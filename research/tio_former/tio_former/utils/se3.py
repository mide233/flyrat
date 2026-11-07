"""Small SE(3) / quaternion utilities used by the simulation and data pipeline.

Quaternions use scalar-first order ``[w, x, y, z]`` throughout this package.
The functions are intentionally dependency-light (NumPy only) so the data
generation path works without SciPy or PyTorch.
"""

from __future__ import annotations

import numpy as np

_EPS = 1e-12


def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < _EPS:
        raise ValueError("cannot normalize a zero vector")
    return v / n


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n < _EPS:
        raise ValueError("cannot normalize a zero quaternion")
    return q / n


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.asarray(a, dtype=np.float64)
    bw, bx, by, bz = np.asarray(b, dtype=np.float64)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    v = np.asarray(v, dtype=np.float64)
    qv = q[1:]
    t = 2.0 * np.cross(qv, v)
    return v + q[0] * t + np.cross(qv, t)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_normalize(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat(m: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to a unit quaternion (scalar first)."""
    m = np.asarray(m, dtype=np.float64)
    if m.shape != (3, 3):
        raise ValueError(f"expected 3x3 rotation, got {m.shape}")
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = quat_normalize(np.array([w, x, y, z], dtype=np.float64))
    if q[0] < 0.0:
        q = -q
    return q


def quat_slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    a = quat_normalize(a)
    b = quat_normalize(b)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        return quat_normalize(a + t * (b - a))
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    s = np.sin(theta)
    return quat_normalize(np.sin((1.0 - t) * theta) / s * a + np.sin(t * theta) / s * b)


def axis_angle_to_quat(axis_angle: np.ndarray) -> np.ndarray:
    axis_angle = np.asarray(axis_angle, dtype=np.float64)
    angle = float(np.linalg.norm(axis_angle))
    if angle < _EPS:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    axis = axis_angle / angle
    half = 0.5 * angle
    return np.array(
        [np.cos(half), *(np.sin(half) * axis)], dtype=np.float64
    )


def quat_to_axis_angle(q: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    if q[0] < 0.0:
        q = -q
    sin_half = float(np.linalg.norm(q[1:]))
    if sin_half < _EPS:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * np.arctan2(sin_half, q[0])
    return q[1:] / sin_half * angle


def so3_exp(axis_angle: np.ndarray) -> np.ndarray:
    return quat_to_matrix(axis_angle_to_quat(axis_angle))


def so3_log(rotation: np.ndarray) -> np.ndarray:
    return quat_to_axis_angle(matrix_to_quat(rotation))


def yaw_to_quat(yaw: float) -> np.ndarray:
    half = 0.5 * yaw
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=np.float64)


def pose_to_matrix(position: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = quat_to_matrix(quaternion)
    t[:3, 3] = np.asarray(position, dtype=np.float64)
    return t


def matrix_to_pose(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    t = np.asarray(t, dtype=np.float64)
    if t.shape != (4, 4):
        raise ValueError(f"expected 4x4 pose, got {t.shape}")
    return t[:3, 3].copy(), matrix_to_quat(t[:3, :3])


def pose_inverse(t: np.ndarray) -> np.ndarray:
    t = np.asarray(t, dtype=np.float64)
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def relative_pose(t_prev: np.ndarray, t_curr: np.ndarray) -> np.ndarray:
    """Return body-frame relative motion ``T_prev^-1 * T_curr``."""
    return pose_inverse(t_prev) @ np.asarray(t_curr, dtype=np.float64)


def interpolate_pose(t0: np.ndarray, t1: np.ndarray, alpha: float) -> np.ndarray:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    p0, q0 = matrix_to_pose(t0)
    p1, q1 = matrix_to_pose(t1)
    return pose_to_matrix(p0 + alpha * (p1 - p0), quat_slerp(q0, q1, alpha))


def integrate_increment(t_prev: np.ndarray, dp: np.ndarray, dphi: np.ndarray) -> np.ndarray:
    """Integrate a body-frame increment using the paper's recursion.

    R_t = R_{t-1} exp([dphi]_x)
    p_t = p_{t-1} + R_{t-1} dp
    """
    t_prev = np.asarray(t_prev, dtype=np.float64)
    r_prev = t_prev[:3, :3]
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = r_prev @ so3_exp(dphi)
    out[:3, 3] = t_prev[:3, 3] + r_prev @ np.asarray(dp, dtype=np.float64)
    return out


def orthonormalize_rotation(rotation: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(rotation)
    r = u @ vt
    if np.linalg.det(r) < 0.0:
        u[:, -1] *= -1.0
        r = u @ vt
    return r
