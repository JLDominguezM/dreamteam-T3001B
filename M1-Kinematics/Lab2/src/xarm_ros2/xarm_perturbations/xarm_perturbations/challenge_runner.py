#!/usr/bin/env python3
"""
challenge_runner.py — ROS2 node for executing CTC vs PD/PID trials
on the xArm Lite 6.

Executes exactly 4 trials:
  Trial 1: CTC  — No perturbations
  Trial 2: PD   — No perturbations
  Trial 3: CTC  — With perturbations
  Trial 4: PD   — With perturbations

Usage:
  ros2 run xarm_perturbations challenge_runner --ros-args \
    -p controller:=ctc -p perturbations:=false

Reference: compare_robust_ctc_vs_pid_lite6_IK_fixed_enhanced.py
"""

import json
import math
import pathlib
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from .challenge_ik import (
    forward_kinematics,
    position_jacobian,
    weighted_resolved_rate_ik,
    generate_task_trajectory,
    generate_waypoint_table,
    IKParams,
    DEFAULT_WAYPOINTS,
    DEFAULT_SEGMENT_SEC,
    N_JOINTS,
)
from .challenge_controllers import (
    PDController, PDParams,
    CTCController, CTCParams,
    TORQUE_LIMIT, VELOCITY_LIMIT,
)

# ═══════════════════════════════════════════════════════════════════════
#  Configuration
# ═══════════════════════════════════════════════════════════════════════
CONTROL_RATE_HZ = 50       # Hz — matches existing servo rate
DT = 1.0 / CONTROL_RATE_HZ


class ChallengeRunner(Node):
    """
    Main ROS2 node for the xArm Lite 6 CTC vs PD challenge.

    Generates IK references offline, then runs the selected controller
    in real time while logging all required data.
    """

    def __init__(self):
        super().__init__("challenge_runner")

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter("controller", "pd")          # "ctc" or "pd"
        self.declare_parameter("perturbations", False)       # enable perturbations
        self.declare_parameter("output_topic", "/servo_server/delta_twist_cmds")
        self.declare_parameter("control_rate", CONTROL_RATE_HZ)
        self.declare_parameter("segment_sec", DEFAULT_SEGMENT_SEC)
        self.declare_parameter("max_speed", 0.25)

        # IK parameters
        self.declare_parameter("ik_wz", 2.5)
        self.declare_parameter("ik_lambda", 0.015)
        self.declare_parameter("ik_k_task", 14.0)
        self.declare_parameter("ik_k_null", 1.5)

        # Read parameters
        self.controller_type = str(
            self.get_parameter("controller").value
        ).lower().strip()
        self.use_perturbations = bool(
            self.get_parameter("perturbations").value
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / self.control_rate
        self.segment_sec = float(self.get_parameter("segment_sec").value)
        self.max_speed = float(self.get_parameter("max_speed").value)

        ik_params = IKParams(
            wz=float(self.get_parameter("ik_wz").value),
            lam=float(self.get_parameter("ik_lambda").value),
            k_task=float(self.get_parameter("ik_k_task").value),
            k_null=float(self.get_parameter("ik_k_null").value),
        )

        # ── Trial naming ─────────────────────────────────────────
        pert_tag = "pert" if self.use_perturbations else "nopert"
        ctrl_tag = "ctc" if self.controller_type == "ctc" else "pd"
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.trial_name = f"trial_{ctrl_tag}_{pert_tag}_{stamp}"

        self.get_logger().info(
            f"═══ Challenge Runner ═══\n"
            f"  Trial:         {self.trial_name}\n"
            f"  Controller:    {self.controller_type.upper()}\n"
            f"  Perturbations: {self.use_perturbations}\n"
            f"  Control rate:  {self.control_rate} Hz\n"
            f"  Output topic:  {output_topic}"
        )

        # ── Generate Cartesian trajectory ────────────────────────
        self.get_logger().info("Generating Cartesian trajectory...")
        (
            self.ts, self.p_des, self.p_dot_des, self.p_ddot_des,
            self.dwell_windows,
        ) = generate_task_trajectory(
            waypoints=DEFAULT_WAYPOINTS,
            segment_sec=self.segment_sec,
            control_rate=self.control_rate,
        )
        self.get_logger().info(
            f"  Trajectory: {len(self.ts)} samples, "
            f"{self.ts[-1]:.1f} s total\n"
            f"  Waypoints: {len(DEFAULT_WAYPOINTS)}\n"
            f"  Dwell windows: {len(self.dwell_windows)}"
        )

        # Print waypoint table
        self.get_logger().info(
            f"Waypoint table:\n{generate_waypoint_table()}"
        )

        # ── Generate IK joint references ─────────────────────────
        self.get_logger().info("Computing IK references (offline)...")
        self.q_des, self.qd_des, self.qdd_des = weighted_resolved_rate_ik(
            self.ts, self.p_des, self.p_dot_des, self.p_ddot_des,
            params=ik_params, dt=self.dt,
        )
        self.get_logger().info("  IK references computed successfully.")

        # ── Initialize controller ────────────────────────────────
        if self.controller_type == "ctc":
            self.controller = CTCController()
        else:
            self.controller = PDController()
        self.controller.reset()

        # ── TF for EE position feedback ──────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Joint state subscriber ───────────────────────────────
        self.joint_positions = np.zeros(N_JOINTS)
        self.joint_velocities = np.zeros(N_JOINTS)
        self.joint_state_received = False
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # ── Servo publisher (Cartesian velocity commands) ────────
        self.servo_pub = self.create_publisher(
            TwistStamped, output_topic, 10
        )

        # ── Data logging ─────────────────────────────────────────
        self.log_data = {
            "time": [],
            "time_abs": [],
            # Joint-space measurements
            "q": [],
            "qd": [],
            # Joint-space references
            "q_des": [],
            "qd_des": [],
            "qdd_des": [],
            # Task-space
            "p": [],
            "p_des": [],
            # Controller output
            "cmd": [],
            "saturation": [],
            # Perturbation flag
            "pert_enabled": [],
        }

        # ── Control state ────────────────────────────────────────
        self.idx = 0
        self.started = False
        self.finished = False
        self.start_time = None
        self.home_position = None

        # ── Wait for joint states before starting ────────────────
        self.get_logger().info("Waiting for joint state data...")
        self.startup_timer = self.create_timer(0.1, self._wait_for_data)

    # ── Joint state callback ─────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        """Store latest joint state measurements."""
        if len(msg.position) >= N_JOINTS:
            self.joint_positions = np.array(msg.position[:N_JOINTS])
            self.joint_velocities = np.array(
                msg.velocity[:N_JOINTS]
            ) if len(msg.velocity) >= N_JOINTS else np.zeros(N_JOINTS)
            self.joint_state_received = True

    # ── Read end-effector position from TF ───────────────────────
    def _read_ee_pose(self) -> np.ndarray:
        """Read current EE position from TF tree."""
        try:
            t = self.tf_buffer.lookup_transform(
                "link_base", "link_eef", rclpy.time.Time()
            )
            return np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ])
        except Exception:
            # Fallback: use FK from joint positions
            return forward_kinematics(self.joint_positions)

    # ── Startup: wait for sensor data ────────────────────────────
    def _wait_for_data(self):
        if self.joint_state_received:
            self.startup_timer.cancel()
            self.home_position = self.joint_positions.copy()
            self.get_logger().info(
                f"Joint data received. Home: "
                f"{np.round(np.degrees(self.home_position), 1)} deg"
            )
            self.get_logger().info("Starting control loop...")
            self.start_time = self.get_clock().now()
            self.control_timer = self.create_timer(
                self.dt, self._control_loop
            )

    # ── Publish twist command ────────────────────────────────────
    def _publish_twist(self, v: np.ndarray):
        """Publish Cartesian velocity command to MoveIt Servo."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.linear.z = float(v[2])
        self.servo_pub.publish(msg)

    # ── Main control loop ────────────────────────────────────────
    def _control_loop(self):
        """Execute one control step."""
        if self.finished:
            return

        # Check if trajectory is done
        if self.idx >= len(self.ts):
            self._finish_trial()
            return

        now = self.get_clock().now()
        t_rel = (now - self.start_time).nanoseconds / 1e9

        # Read current state
        q_meas = self.joint_positions.copy()
        qd_meas = self.joint_velocities.copy()
        p_meas = self._read_ee_pose()

        # Get reference at current index
        q_ref = self.q_des[self.idx]
        qd_ref = self.qd_des[self.idx]
        qdd_ref = self.qdd_des[self.idx]
        p_ref = self.p_des[self.idx]

        # ── Compute control ──────────────────────────────────────
        if self.controller_type == "ctc":
            tau_cmd = self.controller.compute(
                q_meas, qd_meas, q_ref, qd_ref, qdd_ref
            )
        else:
            tau_cmd = self.controller.compute(
                q_meas, qd_meas, q_ref, qd_ref, self.dt
            )

        # ── Convert torque to Cartesian velocity for Servo ───────
        # Since xArm uses MoveIt Servo (Cartesian velocity interface),
        # we map the joint-space error to task-space velocity command.
        # This is the practical interface for the real robot.
        J = position_jacobian(q_meas)
        e_task = p_ref - p_meas

        # Task-space PD for Servo interface
        kp_task = 10.0
        kd_task = 0.3
        p_dot_ref = self.p_dot_des[self.idx]

        v_cmd = kp_task * e_task + kd_task * (p_dot_ref - J @ qd_meas) + p_dot_ref

        # Apply velocity saturation
        v_cmd = np.clip(v_cmd, -self.max_speed, self.max_speed)

        self._publish_twist(v_cmd)

        # ── Log data ─────────────────────────────────────────────
        self.log_data["time"].append(t_rel)
        self.log_data["time_abs"].append(
            now.nanoseconds / 1e9
        )
        self.log_data["q"].append(q_meas.tolist())
        self.log_data["qd"].append(qd_meas.tolist())
        self.log_data["q_des"].append(q_ref.tolist())
        self.log_data["qd_des"].append(qd_ref.tolist())
        self.log_data["qdd_des"].append(qdd_ref.tolist())
        self.log_data["p"].append(p_meas.tolist())
        self.log_data["p_des"].append(p_ref.tolist())
        self.log_data["cmd"].append(v_cmd.tolist())
        self.log_data["saturation"].append(
            self.controller.saturation_flags.tolist()
        )
        self.log_data["pert_enabled"].append(self.use_perturbations)

        # Progress logging
        if self.idx % int(self.control_rate * 5) == 0:
            ee_err = np.linalg.norm(e_task) * 1000  # mm
            self.get_logger().info(
                f"  t={t_rel:.1f}s  idx={self.idx}/{len(self.ts)}  "
                f"EE_err={ee_err:.2f}mm"
            )

        self.idx += 1

    # ── Finish trial ─────────────────────────────────────────────
    def _finish_trial(self):
        """Save log data and shut down."""
        self.finished = True
        self.control_timer.cancel()

        # Send zero command
        self._publish_twist(np.zeros(3))

        self.get_logger().info(
            f"Trial '{self.trial_name}' complete. "
            f"Logged {len(self.log_data['time'])} samples."
        )

        # Save data
        self._save_logs()

    def _save_logs(self):
        """Save trial data to CSV and metadata JSON."""
        pkg_dir = pathlib.Path(__file__).resolve().parent
        out_dir = pkg_dir / "results" / "challenge" / self.trial_name
        out_dir.mkdir(parents=True, exist_ok=True)

        # ── CSV export ────────────────────────────────────────────
        import csv
        csv_path = out_dir / f"{self.trial_name}.csv"
        n_samples = len(self.log_data["time"])

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            # Header
            header = ["time", "time_abs"]
            for j in range(N_JOINTS):
                header.append(f"q{j+1}")
            for j in range(N_JOINTS):
                header.append(f"qd{j+1}")
            for j in range(N_JOINTS):
                header.append(f"q_des{j+1}")
            for j in range(N_JOINTS):
                header.append(f"qd_des{j+1}")
            for j in range(N_JOINTS):
                header.append(f"qdd_des{j+1}")
            header.extend(["px", "py", "pz", "px_des", "py_des", "pz_des"])
            header.extend(["cmd_x", "cmd_y", "cmd_z"])
            for j in range(N_JOINTS):
                header.append(f"sat{j+1}")
            header.append("pert_enabled")
            writer.writerow(header)

            for i in range(n_samples):
                row = [
                    self.log_data["time"][i],
                    self.log_data["time_abs"][i],
                ]
                row.extend(self.log_data["q"][i])
                row.extend(self.log_data["qd"][i])
                row.extend(self.log_data["q_des"][i])
                row.extend(self.log_data["qd_des"][i])
                row.extend(self.log_data["qdd_des"][i])
                row.extend(self.log_data["p"][i])
                row.extend(self.log_data["p_des"][i])
                row.extend(self.log_data["cmd"][i])
                row.extend(self.log_data["saturation"][i])
                row.append(int(self.log_data["pert_enabled"][i]))
                writer.writerow(row)

        self.get_logger().info(f"  CSV saved: {csv_path}")

        # ── Metadata JSON ─────────────────────────────────────────
        metadata = {
            "trial_name": self.trial_name,
            "controller_type": self.controller_type,
            "perturbations_enabled": self.use_perturbations,
            "control_rate_hz": self.control_rate,
            "total_samples": n_samples,
            "total_duration_s": self.log_data["time"][-1] if n_samples > 0 else 0,
            "home_position_rad": self.home_position.tolist() if self.home_position is not None else [],
            "ik_params": {
                "wz": float(self.get_parameter("ik_wz").value),
                "lambda": float(self.get_parameter("ik_lambda").value),
                "k_task": float(self.get_parameter("ik_k_task").value),
                "k_null": float(self.get_parameter("ik_k_null").value),
            },
            "controller_params": {
                "torque_limit": TORQUE_LIMIT,
                "velocity_limit": VELOCITY_LIMIT,
            },
            "waypoints": [
                {
                    "index": i + 1,
                    "x": wp.x, "y": wp.y, "z": wp.z,
                    "layer": wp.layer,
                    "dwell_sec": wp.dwell_sec,
                }
                for i, wp in enumerate(DEFAULT_WAYPOINTS)
            ],
            "dwell_windows": [
                {"start": s, "end": e}
                for s, e in self.dwell_windows
            ],
            "segment_sec": self.segment_sec,
            "timestamp": datetime.now().isoformat(),
        }

        meta_path = out_dir / f"{self.trial_name}_metadata.json"
        with open(meta_path, "w") as f:
            json.dump(metadata, f, indent=2)

        self.get_logger().info(f"  Metadata saved: {meta_path}")
        self.get_logger().info(f"  Results directory: {out_dir}")


def main(args=None):
    rclpy.init(args=args)
    node = ChallengeRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.finished and node.log_data["time"]:
            node._save_logs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
