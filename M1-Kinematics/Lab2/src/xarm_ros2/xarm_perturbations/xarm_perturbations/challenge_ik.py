#!/usr/bin/env python3
"""
challenge_ik.py — Forward Kinematics, Jacobian, and Weighted Resolved-Rate IK
for xArm Lite 6.

Reference: compare_robust_ctc_vs_pid_lite6_IK_fixed_enhanced.py
Components implemented:
  - FK via DH parameters
  - Position Jacobian (3x6) via numerical differentiation
  - Z-priority weighted resolved-rate IK (W = diag(1,1,wz))
  - DLS pseudo-inverse: J# = Jw^T (Jw Jw^T + lambda^2 I)^-1
  - Nullspace posture stabilization
  - Consistent q_des, qd_des, qdd_des generation with Jdot estimation
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple

# ═══════════════════════════════════════════════════════════════════════
#  xArm Lite 6 DH Parameters (Modified DH from URDF)
# ═══════════════════════════════════════════════════════════════════════
# Joint origins from URDF (link_base -> link6):
#   joint1: xyz=(0, 0, 0.2435)     rpy=(0, 0, 0)
#   joint2: xyz=(0, 0, 0)          rpy=(pi/2, -pi/2, pi)
#   joint3: xyz=(0.2002, 0, 0)     rpy=(-pi, 0, pi/2)
#   joint4: xyz=(0.087, -0.22761, 0) rpy=(pi/2, 0, 0)
#   joint5: xyz=(0, 0, 0)          rpy=(pi/2, 0, 0)
#   joint6: xyz=(0, 0.0625, 0)     rpy=(-pi/2, 0, 0)

# Standard DH parameters derived from URDF
# (a, d, alpha, theta_offset)
DH_PARAMS = [
    (0.0,      0.2435,   0.0,       0.0),       # joint1
    (0.0,      0.0,     -np.pi/2,   0.0),       # joint2
    (0.2002,   0.0,      0.0,       0.0),       # joint3
    (0.087,    0.22761, -np.pi/2,   0.0),       # joint4
    (0.0,      0.0,      np.pi/2,   0.0),       # joint5
    (0.0,      0.0625,  -np.pi/2,   0.0),       # joint6
]

N_JOINTS = 6

# Joint limits from URDF (radians)
JOINT_LIMITS_LOWER = np.array([
    -2 * np.pi, -2.61799, -0.061087,
    -2 * np.pi, -2.1642,  -2 * np.pi
])
JOINT_LIMITS_UPPER = np.array([
    2 * np.pi,  2.61799,  5.235988,
    2 * np.pi,  2.1642,   2 * np.pi
])


def _dh_matrix(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Compute 4x4 homogeneous transformation from standard DH parameters."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d],
        [0.0, 0.0,      0.0,     1.0],
    ])


def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """
    Compute end-effector position from joint angles.

    Args:
        q: (6,) array of joint angles [rad]

    Returns:
        p: (3,) array of end-effector position [m]
    """
    T = np.eye(4)
    for i in range(N_JOINTS):
        a, d, alpha, offset = DH_PARAMS[i]
        T = T @ _dh_matrix(a, d, alpha, q[i] + offset)
    return T[:3, 3].copy()


def forward_kinematics_full(q: np.ndarray) -> np.ndarray:
    """
    Compute full 4x4 transformation to end-effector.

    Args:
        q: (6,) array of joint angles [rad]

    Returns:
        T: (4, 4) homogeneous transformation matrix
    """
    T = np.eye(4)
    for i in range(N_JOINTS):
        a, d, alpha, offset = DH_PARAMS[i]
        T = T @ _dh_matrix(a, d, alpha, q[i] + offset)
    return T


def position_jacobian(q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """
    Compute position Jacobian (3x6) via numerical differentiation.

    Args:
        q: (6,) array of joint angles [rad]
        eps: perturbation for finite differences

    Returns:
        J: (3, 6) position Jacobian matrix
    """
    p0 = forward_kinematics(q)
    J = np.zeros((3, N_JOINTS))
    for j in range(N_JOINTS):
        q_pert = q.copy()
        q_pert[j] += eps
        J[:, j] = (forward_kinematics(q_pert) - p0) / eps
    return J


# ═══════════════════════════════════════════════════════════════════════
#  IK Parameters
# ═══════════════════════════════════════════════════════════════════════
@dataclass
class IKParams:
    """Parameters for the weighted resolved-rate IK solver."""
    wz: float = 2.5          # Z-axis weight (> 1.0), recommended [2.0, 3.5]
    lam: float = 1.5e-2      # DLS damping, recommended [0.01, 0.02]
    k_task: float = 14.0     # Task-space error gain, recommended [10, 20]
    k_null: float = 1.5      # Nullspace posture gain, recommended [1.0, 2.5]
    q_home: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    )


def weighted_resolved_rate_ik(
    ts: np.ndarray,
    p_des: np.ndarray,
    p_dot_des: np.ndarray,
    p_ddot_des: np.ndarray,
    params: IKParams = None,
    dt: float = 0.005,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate joint references using weighted resolved-rate IK.

    Implements:
      - Z-priority weighting: W = diag(1, 1, wz), Jw = W @ J
      - DLS pseudo-inverse: J# = Jw^T (Jw Jw^T + lam^2 I)^-1
      - Nullspace posture stabilization:
            q_dot_null = (I - J# Jw)(-k_null (q - q_home))
      - Task-space error feedback:
            q_dot_des = J#(p_dot_des + k_task(p_des - p)) + q_dot_null

    Args:
        ts: (N,) time array
        p_des: (N, 3) desired Cartesian positions
        p_dot_des: (N, 3) desired Cartesian velocities
        p_ddot_des: (N, 3) desired Cartesian accelerations
        params: IK parameters
        dt: time step for integration

    Returns:
        q_des: (N, 6) desired joint positions
        qd_des: (N, 6) desired joint velocities
        qdd_des: (N, 6) desired joint accelerations
    """
    if params is None:
        params = IKParams()

    N = len(ts)
    q_des = np.zeros((N, N_JOINTS))
    qd_des = np.zeros((N, N_JOINTS))
    qdd_des = np.zeros((N, N_JOINTS))

    # Weight matrix for Z-priority
    W = np.diag([1.0, 1.0, params.wz])
    I3 = np.eye(3)
    I6 = np.eye(N_JOINTS)

    # Initialize from home configuration
    q_cur = params.q_home.copy()
    q_des[0] = q_cur.copy()
    qd_prev = np.zeros(N_JOINTS)
    J_prev = None

    for i in range(N):
        # Current FK position
        p_cur = forward_kinematics(q_cur)

        # Compute Jacobian
        J = position_jacobian(q_cur)

        # Weighted Jacobian: Jw = W @ J
        Jw = W @ J

        # DLS pseudo-inverse: J# = Jw^T (Jw Jw^T + lam^2 I)^-1
        JwJwT = Jw @ Jw.T
        J_sharp = Jw.T @ np.linalg.inv(JwJwT + params.lam**2 * I3)

        # Task-space error feedback
        p_err = p_des[i] - p_cur
        xdot_cmd = p_dot_des[i] + params.k_task * p_err

        # Desired joint velocity from task
        qd_task = J_sharp @ xdot_cmd

        # Nullspace posture stabilization
        # q_dot_null = (I - J# Jw)(-k_null (q - q_home))
        N_proj = I6 - J_sharp @ Jw
        qd_null = N_proj @ (-params.k_null * (q_cur - params.q_home))

        # Total desired joint velocity
        qd = qd_task + qd_null

        # Store
        q_des[i] = q_cur.copy()
        qd_des[i] = qd.copy()

        # Estimate qdd using lightweight Jdot approach
        # (finite-difference on joint velocity)
        if i > 0:
            actual_dt = ts[i] - ts[i - 1] if ts[i] > ts[i - 1] else dt
            qdd_des[i] = (qd - qd_prev) / actual_dt
        else:
            qdd_des[i] = np.zeros(N_JOINTS)

        # Integrate forward for next step
        if i < N - 1:
            step_dt = ts[i + 1] - ts[i] if i < N - 1 else dt
            q_cur = q_cur + qd * step_dt
            # Clamp to joint limits
            q_cur = np.clip(q_cur, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

        qd_prev = qd.copy()
        J_prev = J.copy()

    return q_des, qd_des, qdd_des


# ═══════════════════════════════════════════════════════════════════════
#  Trajectory Generation — Quintic spline between waypoints with dwells
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class WaypointDef:
    """Single waypoint definition."""
    x: float
    y: float
    z: float
    layer: str       # "low" or "high"
    dwell_sec: float  # dwell time at waypoint


def quintic_coeffs(
    p0: float, pf: float, v0: float, vf: float,
    a0: float, af: float, T: float
) -> np.ndarray:
    """
    Compute quintic polynomial coefficients for a 1D trajectory segment.
    p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    """
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
    c0 = p0
    c1 = v0
    c2 = a0 / 2.0
    c3 = (20 * (pf - p0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T2) / (2 * T3)
    c4 = (30 * (p0 - pf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T2) / (2 * T4)
    c5 = (12 * (pf - p0) - 6 * (vf + v0) * T - (a0 - af) * T2) / (2 * T5)
    return np.array([c0, c1, c2, c3, c4, c5])


def eval_quintic(coeffs: np.ndarray, t: float):
    """Evaluate quintic polynomial and its derivatives at time t."""
    c = coeffs
    t2, t3, t4, t5 = t**2, t**3, t**4, t**5
    pos = c[0] + c[1]*t + c[2]*t2 + c[3]*t3 + c[4]*t4 + c[5]*t5
    vel = c[1] + 2*c[2]*t + 3*c[3]*t2 + 4*c[4]*t3 + 5*c[5]*t4
    acc = 2*c[2] + 6*c[3]*t + 12*c[4]*t2 + 20*c[5]*t3
    return pos, vel, acc


# ── Default waypoints for the challenge task ─────────────────────────
# 8 distinct waypoints, 2 Z levels (low=0.15, high=0.23), forms a non-trivial
# "figure-8 in layers" path within xArm Lite 6 reachable workspace.
DEFAULT_WAYPOINTS = [
    # Low layer (z = 0.15 m)
    WaypointDef(0.30,  0.10,  0.15, "low",  1.5),   # WP1 — start
    WaypointDef(0.35,  0.05,  0.15, "low",  1.0),   # WP2
    WaypointDef(0.32, -0.08,  0.15, "low",  1.0),   # WP3
    WaypointDef(0.28, -0.10,  0.15, "low",  1.0),   # WP4
    # High layer (z = 0.23 m) — >=0.08 m separation
    WaypointDef(0.28,  0.10,  0.23, "high", 1.0),   # WP5
    WaypointDef(0.33,  0.08,  0.23, "high", 1.0),   # WP6
    WaypointDef(0.35, -0.05,  0.23, "high", 1.0),   # WP7
    WaypointDef(0.30, -0.10,  0.23, "high", 1.0),   # WP8
    # Return to WP1 (close the loop)
    WaypointDef(0.30,  0.10,  0.15, "low",  1.5),   # WP9 = WP1
]

DEFAULT_SEGMENT_SEC = 2.0  # transition time between waypoints


def generate_task_trajectory(
    waypoints: list = None,
    segment_sec: float = None,
    control_rate: float = 200.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, list]:
    """
    Generate smooth Cartesian trajectory with dwells at each waypoint.

    Uses quintic splines for transitions (zero vel/acc at boundaries)
    and constant-position dwells for steady-state evaluation.

    Args:
        waypoints: list of WaypointDef
        segment_sec: transition time between waypoints
        control_rate: Hz

    Returns:
        ts: (N,) time array
        p_des: (N, 3) desired positions
        p_dot_des: (N, 3) desired velocities
        p_ddot_des: (N, 3) desired accelerations
        dwell_windows: list of (t_start, t_end) for each waypoint dwell
    """
    if waypoints is None:
        waypoints = DEFAULT_WAYPOINTS
    if segment_sec is None:
        segment_sec = DEFAULT_SEGMENT_SEC

    dt = 1.0 / control_rate
    n_wp = len(waypoints)

    # Build time segments: [transition, dwell, transition, dwell, ...]
    segments = []
    for i in range(n_wp):
        if i > 0:
            segments.append(("transition", segment_sec, i - 1, i))
        segments.append(("dwell", waypoints[i].dwell_sec, i, i))

    # Total duration
    total_time = sum(s[1] for s in segments)
    N = int(total_time * control_rate) + 1
    ts = np.linspace(0.0, total_time, N)

    p_des = np.zeros((N, 3))
    p_dot_des = np.zeros((N, 3))
    p_ddot_des = np.zeros((N, 3))
    dwell_windows = []

    t_offset = 0.0
    for seg_type, seg_dur, idx_from, idx_to in segments:
        wp_from = waypoints[idx_from]
        wp_to = waypoints[idx_to]

        p_start = np.array([wp_from.x, wp_from.y, wp_from.z])
        p_end = np.array([wp_to.x, wp_to.y, wp_to.z])

        # Find indices in ts that fall within this segment
        t_seg_start = t_offset
        t_seg_end = t_offset + seg_dur
        mask = (ts >= t_seg_start - 1e-9) & (ts < t_seg_end + 1e-9)
        idxs = np.where(mask)[0]

        if seg_type == "dwell":
            # Constant position, zero velocity/acceleration
            for k in idxs:
                p_des[k] = p_end.copy()
                p_dot_des[k] = 0.0
                p_ddot_des[k] = 0.0
            dwell_windows.append((t_seg_start, t_seg_end))

        else:  # transition — quintic spline
            T = seg_dur
            # Compute quintic coefficients per axis
            coeffs_xyz = []
            for ax in range(3):
                c = quintic_coeffs(
                    p_start[ax], p_end[ax],
                    0.0, 0.0, 0.0, 0.0, T
                )
                coeffs_xyz.append(c)

            for k in idxs:
                t_local = ts[k] - t_seg_start
                t_local = np.clip(t_local, 0.0, T)
                for ax in range(3):
                    pos, vel, acc = eval_quintic(coeffs_xyz[ax], t_local)
                    p_des[k, ax] = pos
                    p_dot_des[k, ax] = vel
                    p_ddot_des[k, ax] = acc

        t_offset += seg_dur

    return ts, p_des, p_dot_des, p_ddot_des, dwell_windows


def generate_waypoint_table(waypoints: list = None, segment_sec: float = None) -> str:
    """Generate a printable waypoint table for documentation."""
    if waypoints is None:
        waypoints = DEFAULT_WAYPOINTS
    if segment_sec is None:
        segment_sec = DEFAULT_SEGMENT_SEC

    lines = [
        "| WP# | X (m)  | Y (m)   | Z (m)  | Layer | Dwell (s) | Trans (s) |",
        "|-----|--------|---------|--------|-------|-----------|-----------|",
    ]
    for i, wp in enumerate(waypoints):
        trans = segment_sec if i < len(waypoints) - 1 else 0.0
        lines.append(
            f"|  {i+1:2d} | {wp.x:.3f} | {wp.y:+.3f}  | {wp.z:.3f} "
            f"| {wp.layer:4s}  |   {wp.dwell_sec:.1f}     |   {trans:.1f}     |"
        )
    return "\n".join(lines)
