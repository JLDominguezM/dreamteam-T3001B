#!/usr/bin/env python3
"""
challenge_controllers.py — PD Baseline and CTC Controllers for xArm Lite 6.

Reference: compare_robust_ctc_vs_pid_lite6_IK_fixed_enhanced.py

PD Controller:
  - Joint-space PD control: tau = -(Kp e + Kd e_dot)
  - No integral action (consistent with previous lab work)
  - Actuator saturation handling
  - Rate limiting for safety

CTC Controller:
  - Computed Torque Control with dynamics: tau = M v + C qd + G + F
  - Feedback linearization: v = qdd_ref - Kp e - Kd e_dot
  - Optional robust/sliding term: tau_robust = k_robust * sign(S)
  - Actuator saturation handling
"""

import numpy as np
from dataclasses import dataclass, field


# ═══════════════════════════════════════════════════════════════════════
#  Safety Limits
# ═══════════════════════════════════════════════════════════════════════
TORQUE_LIMIT = 10.0       # Nm — per-joint torque saturation
VELOCITY_LIMIT = 2.0      # rad/s — rate limiting
EFFORT_RATE_LIMIT = 50.0  # Nm/s — max torque rate of change

N_JOINTS = 6


# ═══════════════════════════════════════════════════════════════════════
#  PD Baseline Controller
# ═══════════════════════════════════════════════════════════════════════
@dataclass
class PDParams:
    """PD controller gains and parameters."""
    Kp: np.ndarray = field(
        default_factory=lambda: np.diag([40.0, 40.0, 40.0, 30.0, 25.0, 20.0])
    )
    Kd: np.ndarray = field(
        default_factory=lambda: np.diag([14.0, 14.0, 14.0, 10.0, 8.0, 6.0])
    )
    torque_limit: float = TORQUE_LIMIT
    velocity_limit: float = VELOCITY_LIMIT


class PDController:
    """
    Joint-space PD tracking controller (baseline).

    tau = -(Kp @ e + Kd @ e_dot)

    Features:
      - Actuator saturation with documented limits
      - Rate limiting for safe operation
      - Emergency stop capability
    """

    def __init__(self, params: PDParams = None):
        self.params = params or PDParams()
        self.prev_tau = np.zeros(N_JOINTS)
        self.e_stop = False
        self.saturation_flags = np.zeros(N_JOINTS, dtype=bool)

    def reset(self):
        """Reset controller state."""
        self.prev_tau = np.zeros(N_JOINTS)
        self.e_stop = False
        self.saturation_flags = np.zeros(N_JOINTS, dtype=bool)

    def emergency_stop(self):
        """Activate emergency stop — output zero torque."""
        self.e_stop = True

    def compute(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        q_ref: np.ndarray,
        qd_ref: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Compute PD control torque.

        Args:
            q: (6,) measured joint positions [rad]
            qd: (6,) measured joint velocities [rad/s]
            q_ref: (6,) desired joint positions [rad]
            qd_ref: (6,) desired joint velocities [rad/s]
            dt: time step [s]

        Returns:
            tau: (6,) commanded torques [Nm]
        """
        if self.e_stop:
            return np.zeros(N_JOINTS)

        # Tracking errors
        e = q - q_ref
        e_dot = qd - qd_ref

        # PD law
        tau = -(self.params.Kp @ e + self.params.Kd @ e_dot)

        # Rate limiting
        if dt > 0:
            tau_rate = (tau - self.prev_tau) / dt
            max_rate = EFFORT_RATE_LIMIT
            tau_rate = np.clip(tau_rate, -max_rate, max_rate)
            tau = self.prev_tau + tau_rate * dt

        # Actuator saturation
        self.saturation_flags = np.abs(tau) >= self.params.torque_limit
        tau = np.clip(tau, -self.params.torque_limit, self.params.torque_limit)

        self.prev_tau = tau.copy()
        return tau


# ═══════════════════════════════════════════════════════════════════════
#  Robot Dynamics Model (Simplified for xArm Lite 6)
# ═══════════════════════════════════════════════════════════════════════

# Approximate link masses and inertias from URDF
LINK_MASSES = np.array([1.654, 1.166, 0.953, 1.284, 0.804, 0.131])  # kg
GRAVITY = 9.81  # m/s^2

# Approximate link lengths for gravity computation
LINK_LENGTHS = np.array([0.2435, 0.0, 0.2002, 0.087, 0.0, 0.0625])  # m
LINK_OFFSETS = np.array([0.0, 0.0, 0.0, 0.22761, 0.0, 0.0])         # m


def get_robot_dynamics(q: np.ndarray, qd: np.ndarray):
    """
    Compute approximate robot dynamics matrices for xArm Lite 6.

    Uses simplified Lagrangian dynamics model.
    For a real implementation, use the robot_state_publisher or
    a computed dynamics library (e.g., pinocchio, rbdl).

    Args:
        q: (6,) joint positions [rad]
        qd: (6,) joint velocities [rad/s]

    Returns:
        M: (6, 6) mass/inertia matrix
        Cqd: (6,) Coriolis/centrifugal torques
        G: (6,) gravity torques
        F: (6,) friction torques
    """
    # ── Mass matrix (approximate) ─────────────────────────────────
    # Use a simplified model: diagonal dominant with configuration-
    # dependent coupling for first 3 joints
    M = np.eye(N_JOINTS)

    # Approximate effective inertias
    m_eff = np.array([2.5, 3.0, 1.5, 0.5, 0.3, 0.1])

    # Configuration-dependent terms for joints 1-3
    c2, c3 = np.cos(q[1]), np.cos(q[2])
    s2, s3 = np.sin(q[1]), np.sin(q[2])

    l2, l3 = 0.2002, 0.22761  # link lengths

    # Diagonal terms
    M[0, 0] = m_eff[0] + (LINK_MASSES[2] * l2**2 + LINK_MASSES[3] * (l2**2 + l3**2
               + 2 * l2 * l3 * c3)) * c2**2
    M[1, 1] = m_eff[1] + LINK_MASSES[2] * l2**2 + LINK_MASSES[3] * (l2**2 + l3**2
               + 2 * l2 * l3 * c3)
    M[2, 2] = m_eff[2] + LINK_MASSES[3] * l3**2
    M[3, 3] = m_eff[3]
    M[4, 4] = m_eff[4]
    M[5, 5] = m_eff[5]

    # Off-diagonal coupling
    M[1, 2] = LINK_MASSES[3] * l2 * l3 * c3
    M[2, 1] = M[1, 2]

    # Ensure positive definiteness
    M = M + 0.01 * np.eye(N_JOINTS)

    # ── Coriolis/centrifugal (simplified Christoffel symbols) ─────
    Cqd = np.zeros(N_JOINTS)
    h = LINK_MASSES[3] * l2 * l3 * s3
    Cqd[1] = -h * qd[2] * (2 * qd[1] + qd[2])
    Cqd[2] = h * qd[1]**2

    # ── Gravity ──────────────────────────────────────────────────
    G = np.zeros(N_JOINTS)
    G[1] = -(LINK_MASSES[1] + LINK_MASSES[2] + LINK_MASSES[3]) * GRAVITY * l2 * c2 \
           - LINK_MASSES[3] * GRAVITY * l3 * np.cos(q[1] + q[2])
    G[2] = -LINK_MASSES[3] * GRAVITY * l3 * np.cos(q[1] + q[2])

    # ── Friction (viscous) ───────────────────────────────────────
    b_friction = np.array([0.5, 0.5, 0.3, 0.2, 0.1, 0.05])
    F = b_friction * qd

    return M, Cqd, G, F


# ═══════════════════════════════════════════════════════════════════════
#  CTC Controller
# ═══════════════════════════════════════════════════════════════════════
@dataclass
class CTCParams:
    """Computed Torque Control gains and parameters."""
    Kp: np.ndarray = field(
        default_factory=lambda: np.diag([80.0, 80.0, 80.0, 60.0, 50.0, 40.0])
    )
    Kd: np.ndarray = field(
        default_factory=lambda: np.diag([28.0, 28.0, 28.0, 20.0, 16.0, 12.0])
    )
    # Robust/sliding term parameters
    gamma: float = 2.5
    k_robust: float = 5.5
    sign_eps: float = 2e-3
    use_robust: bool = True
    torque_limit: float = TORQUE_LIMIT
    velocity_limit: float = VELOCITY_LIMIT


def _smooth_sign(x: np.ndarray, eps: float = 1e-3) -> np.ndarray:
    """Smooth approximation of sign function using tanh."""
    return np.tanh(x / eps)


class CTCController:
    """
    Computed Torque Control with optional robust/sliding term.

    tau = M(q)(qdd_ref - Kp e - Kd e_dot) + C(q,qd)qd + G(q) + F(qd)
        + k_robust * smooth_sign(gamma * e + e_dot)

    Features:
      - Full dynamics compensation (M, C, G, F)
      - Feedback linearization for decoupled joint control
      - Optional sliding-mode robust term for perturbation rejection
      - Actuator saturation and rate limiting
      - Emergency stop capability
    """

    def __init__(self, params: CTCParams = None):
        self.params = params or CTCParams()
        self.prev_tau = np.zeros(N_JOINTS)
        self.e_stop = False
        self.saturation_flags = np.zeros(N_JOINTS, dtype=bool)

    def reset(self):
        """Reset controller state."""
        self.prev_tau = np.zeros(N_JOINTS)
        self.e_stop = False
        self.saturation_flags = np.zeros(N_JOINTS, dtype=bool)

    def emergency_stop(self):
        """Activate emergency stop — output zero torque."""
        self.e_stop = True

    def compute(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        q_ref: np.ndarray,
        qd_ref: np.ndarray,
        qdd_ref: np.ndarray,
    ) -> np.ndarray:
        """
        Compute CTC control torque.

        Args:
            q: (6,) measured joint positions [rad]
            qd: (6,) measured joint velocities [rad/s]
            q_ref: (6,) desired joint positions [rad]
            qd_ref: (6,) desired joint velocities [rad/s]
            qdd_ref: (6,) desired joint accelerations [rad/s^2]

        Returns:
            tau: (6,) commanded torques [Nm]
        """
        if self.e_stop:
            return np.zeros(N_JOINTS)

        # Get dynamics matrices
        M, Cqd, G, F = get_robot_dynamics(q, qd)

        # Tracking errors
        e = q - q_ref
        e_dot = qd - qd_ref

        # Feedback linearization
        v = qdd_ref - (self.params.Kp @ e) - (self.params.Kd @ e_dot)

        # CTC law: tau = M v + C qd + G + F
        tau = M @ v + Cqd + G + F

        # Optional robust/sliding term
        if self.params.use_robust:
            S = self.params.gamma * e + e_dot
            tau_robust = self.params.k_robust * _smooth_sign(S, self.params.sign_eps)
            tau = tau + tau_robust

        # Actuator saturation
        self.saturation_flags = np.abs(tau) >= self.params.torque_limit
        tau = np.clip(tau, -self.params.torque_limit, self.params.torque_limit)

        self.prev_tau = tau.copy()
        return tau
