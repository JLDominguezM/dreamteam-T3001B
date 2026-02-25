#!/usr/bin/env python3
import math
import pathlib
from datetime import datetime

import numpy as np
import pandas as pd
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class PositionController(Node):

    def __init__(self):
        super().__init__("position_controller")

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter("output_topic", "/servo_server/delta_twist_cmds")

        self.declare_parameter("kp", [8.0, 7.0, 3.0])
        self.declare_parameter("kd", [0.4, 0.4, 0.35])
        self.declare_parameter("ki", [0.0, 0.0, 0.0])

        self.declare_parameter("max_speed", 0.25)
        self.declare_parameter("deadband", 0.001)
        self.declare_parameter("integral_max", 0.05)
        self.declare_parameter("derivative_filter_freq", 8.0)
        self.declare_parameter("radius", 0.08)
        self.declare_parameter("frequency", 0.06)
        self.declare_parameter("soft_start_duration", 3.0)

        output_topic = str(self.get_parameter("output_topic").value)
        self.kp = np.array(self.get_parameter("kp").value, dtype=float)
        self.kd = np.array(self.get_parameter("kd").value, dtype=float)
        self.ki = np.array(self.get_parameter("ki").value, dtype=float)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.epsilon = np.full(3, float(self.get_parameter("deadband").value))
        self.integral_max = float(self.get_parameter("integral_max").value)
        self.radius = float(self.get_parameter("radius").value)
        self.frequency = float(self.get_parameter("frequency").value)
        self.soft_start_dur = float(self.get_parameter("soft_start_duration").value)

        fc = float(self.get_parameter("derivative_filter_freq").value)
        self.tau_d = 1.0 / (2.0 * math.pi * fc)

        # ── PID State ─────────────────────────────────────────────
        self.prev_error = np.zeros(3, dtype=float)
        self.integral = np.zeros(3, dtype=float)
        self.d_filtered = np.zeros(3, dtype=float)

        # ── General State ─────────────────────────────────────────
        self.center = None
        self.start_time = self.get_clock().now()
        self.prev_time = self.get_clock().now()
        self.history_data = []

        # ── TF ────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Publisher ─────────────────────────────────────────────
        self.servo_pub = self.create_publisher(TwistStamped, output_topic, 10)

        # ── Control loop at 50 Hz ─────────────────────────────────
        self.timer = self.create_timer(0.02, self._control_loop)

        self.get_logger().info(
            f"PID Controller ready\n"
            f"  output_topic : {output_topic}\n"
            f"  kp={list(self.kp)}  kd={list(self.kd)}  ki={list(self.ki)}\n"
            f"  max_speed={self.max_speed}  deadband={self.epsilon[0]}\n"
            f"  derivative_filter_freq={fc} Hz  (tau={self.tau_d*1000:.1f} ms)\n"
            f"  integral_max={self.integral_max}\n"
            f"  trajectory: lemniscate  radius={self.radius}  freq={self.frequency}"
        )

    # ── Helpers ────────────────────────────────────────────────────
    def _read_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "link_base", "link_eef", rclpy.time.Time()
            )
            return np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ], dtype=float)
        except Exception:
            return None

    def _lemniscate(self, t_sec: float) -> np.ndarray:
        """Return desired position on lemniscate with soft-start ramp."""
        cx, cy, cz = self.center
        w = 2.0 * math.pi * self.frequency

        # Soft start: cosine ramp 0 → 1
        T = self.soft_start_dur
        s = 0.5 * (1.0 - math.cos(math.pi * t_sec / T)) if t_sec < T else 1.0

        x_rel = self.radius * math.sin(w * t_sec)
        y_rel = self.radius * math.sin(2 * w * t_sec) / 2.0

        return np.array([cx + s * x_rel, cy + s * y_rel, cz])

    def _publish_twist(self, v: np.ndarray):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "link_base"
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.linear.z = float(v[2])
        self.servo_pub.publish(msg)

    # ── Control loop ───────────────────────────────────────────────
    def _control_loop(self):
        current = self._read_pose()
        if current is None:
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        dt = max(dt, 1e-6)
        dt = min(dt, 0.10)

        if self.center is None:
            self.center = current.copy()
            self.start_time = now
            self.get_logger().info("Center recorded. Starting trajectory.")

        t = (now - self.start_time).nanoseconds / 1e9
        desired = self._lemniscate(t)
        error = desired - current

        # ── Filtered derivative ───────────────────────────────────
        alpha = dt / (self.tau_d + dt)
        d_raw = (error - self.prev_error) / dt
        self.d_filtered = alpha * d_raw + (1.0 - alpha) * self.d_filtered

        # ── Integral with anti-windup ─────────────────────────────
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.integral_max, self.integral_max)

        # ── PID command ───────────────────────────────────────────
        v = self.kp * error + self.kd * self.d_filtered + self.ki * self.integral

        # ── Deadband ──────────────────────────────────────────────
        v = np.where(np.abs(error) > self.epsilon, v, 0.0)

        # ── Velocity saturation ───────────────────────────────────
        v = np.clip(v, -self.max_speed, self.max_speed)

        self._publish_twist(v)

        # ── Record data ───────────────────────────────────────────
        self.history_data.append({
            "time": t,
            "des_x": desired[0], "des_y": desired[1], "des_z": desired[2],
            "act_x": current[0], "act_y": current[1], "act_z": current[2],
            "err_x": error[0],   "err_y": error[1],   "err_z": error[2],
            "cmd_x": v[0],       "cmd_y": v[1],        "cmd_z": v[2],
            "vel_mag": float(np.linalg.norm(v)),
        })

        self.prev_error = error.copy()
        self.prev_time = now


# ── Results saving ────────────────────────────────────────────────────
def _get_results_dir() -> pathlib.Path:
    """Create results/<YYYY-MM-DD_HH-MM>/ inside the source package."""
    pkg = pathlib.Path(__file__).resolve()
    src_dir = pkg.parent
    for parent in pkg.parents:
        if parent.name == "install":
            candidate = (
                parent.parent / "src" / "xarm_ros2"
                / "xarm_perturbations" / "xarm_perturbations"
            )
            if candidate.exists():
                src_dir = candidate
            break

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M")
    out = src_dir / "results" / stamp
    out.mkdir(parents=True, exist_ok=True)
    return out


def _save_results(node: PositionController):
    """Compute metrics, save CSV + metrics.txt + 4 plots."""
    df = pd.DataFrame(node.history_data)
    out = _get_results_dir()

    # ── CSV ────────────────────────────────────────────────────────
    csv_path = out / "robot_evaluation.csv"
    df.to_csv(csv_path, index=False)

    # ── Metrics ───────────────────────────────────────────────────
    t = df["time"].to_numpy()
    ex, ey, ez = df["err_x"].to_numpy(), df["err_y"].to_numpy(), df["err_z"].to_numpy()
    e_total = np.sqrt(ex**2 + ey**2 + ez**2)

    rmse_x = float(np.sqrt(np.mean(ex**2)))
    rmse_y = float(np.sqrt(np.mean(ey**2)))
    rmse_z = float(np.sqrt(np.mean(ez**2)))
    rmse_tot = float(np.sqrt(np.mean(e_total**2)))
    max_err_x = float(np.max(np.abs(ex)))
    max_err_y = float(np.max(np.abs(ey)))
    max_err_z = float(np.max(np.abs(ez)))
    max_err_3d = float(np.max(e_total))

    node.get_logger().info(
        f"RMSE  x={rmse_x:.5f}  y={rmse_y:.5f}  z={rmse_z:.5f}  "
        f"total={rmse_tot:.5f} m | Max3D={max_err_3d:.5f} m"
    )

    with open(out / "robot_metrics.txt", "w") as f:
        f.write("=== EVALUATION RESULTS ===\n")
        f.write(f"RMSE_x:          {rmse_x:.6f} m\n")
        f.write(f"RMSE_y:          {rmse_y:.6f} m\n")
        f.write(f"RMSE_z:          {rmse_z:.6f} m\n")
        f.write(f"RMSE_total:      {rmse_tot:.6f} m\n")
        f.write(f"Max_abs_err_x:   {max_err_x:.6f} m\n")
        f.write(f"Max_abs_err_y:   {max_err_y:.6f} m\n")
        f.write(f"Max_abs_err_z:   {max_err_z:.6f} m\n")
        f.write(f"Max_abs_err_3D:  {max_err_3d:.6f} m\n")
        f.write(f"Samples:         {len(df)}\n")
        f.write(f"Duration:        {t[-1]:.2f} s\n")

    # ── Plot style ────────────────────────────────────────────────
    plt.style.use("bmh")

    # Extract all columns as numpy arrays for plotting
    des_x = df["des_x"].to_numpy()
    des_y = df["des_y"].to_numpy()
    des_z = df["des_z"].to_numpy()
    act_x = df["act_x"].to_numpy()
    act_y = df["act_y"].to_numpy()
    act_z = df["act_z"].to_numpy()
    cmd_x = df["cmd_x"].to_numpy()
    cmd_y = df["cmd_y"].to_numpy()
    cmd_z = df["cmd_z"].to_numpy()
    vel_mag = df["vel_mag"].to_numpy()

    # ── Plot 1: Trajectory XY ─────────────────────────────────────
    fig1, ax1 = plt.subplots(figsize=(8, 7))
    ax1.plot(des_x, des_y, "b--", lw=1.5, label="Desired")
    ax1.plot(act_x, act_y, "r-", lw=1.0, label="Actual")
    ax1.plot(des_x[0], des_y[0], "go", ms=8, label="Start")
    ax1.set_xlabel("X [m]"); ax1.set_ylabel("Y [m]")
    ax1.set_title("XY Trajectory — Desired vs Actual (Lemniscate)")
    ax1.legend(); ax1.axis("equal"); ax1.grid(True)
    fig1.tight_layout()
    fig1.savefig(out / "plot_trajectory_xy.png", dpi=150)
    plt.close(fig1)

    # ── Plot 2: Desired vs Actual per axis ────────────────────────
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for i, (label, dc, ac) in enumerate([
        ("X", "des_x", "act_x"), ("Y", "des_y", "act_y"), ("Z", "des_z", "act_z")
    ]):
        axes2[i].plot(t, df[dc].to_numpy(), "b--", lw=1.2, label=f"Desired {label}")
        axes2[i].plot(t, df[ac].to_numpy(), "r-", lw=0.8, label=f"Actual {label}")
        axes2[i].set_ylabel(f"{label} [m]")
        axes2[i].legend(loc="upper right"); axes2[i].grid(True)
    axes2[2].set_xlabel("Time [s]")
    axes2[0].set_title("Desired vs Actual Position per Axis")
    fig2.tight_layout()
    fig2.savefig(out / "plot_desired_vs_actual.png", dpi=150)
    plt.close(fig2)

    # ── Plot 3: Error over time ───────────────────────────────────
    fig3, ax3 = plt.subplots(figsize=(12, 5))
    ax3.plot(t, ex * 1000, label="err_x")
    ax3.plot(t, ey * 1000, label="err_y")
    ax3.plot(t, ez * 1000, label="err_z")
    ax3.plot(t, e_total * 1000, "k--", lw=0.8, label="||error||")
    ax3.set_xlabel("Time [s]"); ax3.set_ylabel("Error [mm]")
    ax3.set_title(
        f"Position Error — RMSE_total={rmse_tot*1000:.2f} mm, "
        f"Max={max_err_3d*1000:.2f} mm"
    )
    ax3.legend(); ax3.grid(True)
    fig3.tight_layout()
    fig3.savefig(out / "plot_error.png", dpi=150)
    plt.close(fig3)

    # ── Plot 4: Commanded velocity magnitude ──────────────────────
    fig4, ax4 = plt.subplots(figsize=(12, 5))
    ax4.plot(t, cmd_x, label="cmd_x")
    ax4.plot(t, cmd_y, label="cmd_y")
    ax4.plot(t, cmd_z, label="cmd_z")
    ax4.plot(t, vel_mag, "k--", lw=0.8, label="||vel||")
    ax4.axhline(y=node.max_speed, color="red", ls=":", lw=1.0, label=f"max_speed={node.max_speed}")
    ax4.set_xlabel("Time [s]"); ax4.set_ylabel("Velocity [m/s]")
    ax4.set_title("Commanded Velocity")
    ax4.legend(); ax4.grid(True)
    fig4.tight_layout()
    fig4.savefig(out / "plot_velocity.png", dpi=150)
    plt.close(fig4)

    node.get_logger().info(f"Results saved to '{out}'")


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.history_data:
            _save_results(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
