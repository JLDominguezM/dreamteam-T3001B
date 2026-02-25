#!/usr/bin/env python3
import math
import pathlib
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class PID:
    def __init__(self, kp: float, kd: float, ki: float,
                 deadband: float = 0.0, alpha_d: float = 0.2,
                 integral_clamp: float = 0.1):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.deadband       = deadband
        self.alpha_d        = alpha_d
        self.integral_clamp = integral_clamp
        self._prev_error  = 0.0
        self._integral    = 0.0
        self._prev_d_filt = 0.0

    def reset(self):
        self._prev_error  = 0.0
        self._integral    = 0.0
        self._prev_d_filt = 0.0

    def compute(self, raw_error: float, dt: float) -> tuple[float, float]:
        if self.deadband > 0.0 and abs(raw_error) < self.deadband:
            error = 0.0
        elif self.deadband > 0.0:
            error = raw_error - math.copysign(self.deadband, raw_error)
        else:
            error = raw_error

        p = self.kp * error

        raw_d  = (error - self._prev_error) / dt
        d_filt = self.alpha_d * raw_d + (1.0 - self.alpha_d) * self._prev_d_filt
        d      = self.kd * d_filt

        v_prelim   = p + d + self.ki * self._integral
        saturating = abs(v_prelim) >= 1e6
        same_sign  = math.copysign(1, error) == math.copysign(1, v_prelim)
        if abs(error) > 0.0 and not (saturating and same_sign):
            self._integral += error * dt
        self._integral    = max(-self.integral_clamp,
                                min(self.integral_clamp, self._integral))
        i = self.ki * self._integral

        out               = p + i + d
        self._prev_error  = error
        self._prev_d_filt = d_filt
        return out, error


class CascadePID:
    def __init__(self,
                 kp_pos: float, kd_pos: float, ki_pos: float,
                 kp_vel: float, kd_vel: float,
                 deadband: float = 0.002,
                 alpha_vel: float = 0.08,
                 max_slew: float = 0.005,
                 alpha_out: float = 0.15):
        self.outer = PID(kp_pos, kd_pos, ki_pos,
                         deadband=deadband, alpha_d=0.15, integral_clamp=0.05)
        # Inner: solo P proporcional, sin derivada (ruidosa sobre vel estimada)
        self.inner = PID(kp_vel, 0.0, 0.0,
                         deadband=0.0, alpha_d=0.30)
        self.alpha_vel     = alpha_vel   # filtro para vel estimada
        self.max_slew      = max_slew    # máx cambio de comando por tick [m/s]
        self.alpha_out     = alpha_out   # filtro EMA en salida (0.1=muy suave, 1.0=sin filtro)
        self._vel_est      = 0.0
        self._prev_act_pos = None
        self._prev_cmd     = 0.0
        self._smooth_cmd   = 0.0

    def reset(self):
        self.outer.reset()
        self.inner.reset()
        self._vel_est      = 0.0
        self._prev_act_pos = None
        self._prev_cmd     = 0.0
        self._smooth_cmd   = 0.0

    def compute(self, pos_error: float, actual_pos: float,
                vel_ff: float, dt: float) -> tuple[float, float, float]:
        """Retorna (comando_velocidad, vel_deseada, vel_estimada)"""
        # Velocidad estimada con filtro EMA agresivo (anti-ruido)
        if self._prev_act_pos is None:
            self._prev_act_pos = actual_pos
        raw_vel       = (actual_pos - self._prev_act_pos) / dt
        self._vel_est = self.alpha_vel * raw_vel + (1.0 - self.alpha_vel) * self._vel_est
        self._prev_act_pos = actual_pos

        # Loop externo: posición → corrección de velocidad
        vel_corr, _  = self.outer.compute(pos_error, dt)
        vel_desired  = vel_ff + vel_corr

        # Loop interno: solo P sobre error de velocidad
        vel_error    = vel_desired - self._vel_est
        command, _   = self.inner.compute(vel_error, dt)

        # Rate limiter: limitar cambio máximo por tick
        delta = command - self._prev_cmd
        if abs(delta) > self.max_slew:
            command = self._prev_cmd + math.copysign(self.max_slew, delta)
        self._prev_cmd = command

        # Filtro EMA en salida: suaviza picos residuales
        self._smooth_cmd = self.alpha_out * command + (1.0 - self.alpha_out) * self._smooth_cmd

        return self._smooth_cmd, vel_desired, self._vel_est


class PositionController(Node):
    def __init__(self):
        super().__init__("position_controller")

        self.declare_parameter('output_topic', '/servo_server/delta_twist_cmds')

        self.declare_parameter('kp_pos_x', 0.5)
        self.declare_parameter('kp_pos_y', 0.9)
        self.declare_parameter('kp_pos_z', 0.5)

        self.declare_parameter('kd_pos_x', 0.001)
        self.declare_parameter('kd_pos_y', 0.001)
        self.declare_parameter('kd_pos_z', 0.01)

        self.declare_parameter('ki_pos_x', 0.0)
        self.declare_parameter('ki_pos_y', 0.0)
        self.declare_parameter('ki_pos_z', 0.0)

        self.declare_parameter('kp_vel_x', 0.5)
        self.declare_parameter('kp_vel_y', 0.5)
        self.declare_parameter('kp_vel_z', 0.5)

        self.declare_parameter('max_speed', 0.30)
        self.declare_parameter('deadband',  0.002)
        self.declare_parameter('radius',    0.06)
        self.declare_parameter('frequency', 0.06)

        output_topic   = self.get_parameter('output_topic').value
        self.max_speed = float(self.get_parameter('max_speed').value)
        deadband       = float(self.get_parameter('deadband').value)
        self.radius    = float(self.get_parameter('radius').value)
        self.frequency = float(self.get_parameter('frequency').value)

        kp_pos = [self.get_parameter(f'kp_pos_{a}').value for a in ('x', 'y', 'z')]
        kd_pos = [self.get_parameter(f'kd_pos_{a}').value for a in ('x', 'y', 'z')]
        ki_pos = [self.get_parameter(f'ki_pos_{a}').value for a in ('x', 'y', 'z')]
        kp_vel = [self.get_parameter(f'kp_vel_{a}').value for a in ('x', 'y', 'z')]

        self._cascades = [
            CascadePID(kp_pos[i], kd_pos[i], ki_pos[i],
                       kp_vel[i], 0.0, deadband)
            for i in range(3)
        ]

        self.servo_pub   = self.create_publisher(TwistStamped, output_topic, 10)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.history_data = []
        self.center     = None
        self.start_time = self.get_clock().now()
        self.prev_time  = self.get_clock().now()

        self.timer = self.create_timer(0.02, self._control_loop)
        self.get_logger().info(
            f"Cascada FF+PID iniciada | "
            f"Kp_pos={kp_pos} Kd_pos={kd_pos} | "
            f"Kp_vel={kp_vel}"
        )

    def _read_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            return np.array([t.transform.translation.x,
                             t.transform.translation.y,
                             t.transform.translation.z])
        except Exception:
            return None

    def _lemniscate(self, t_sec: float) -> tuple[np.ndarray, np.ndarray]:
        """Posición + velocidad feedforward analítica (derivada exacta)."""
        cx, cy, cz = self.center
        w = 2.0 * math.pi * self.frequency
        pos = np.array([
            cx + self.radius * math.sin(w * t_sec),
            cy + self.radius * math.sin(w * t_sec) * math.cos(w * t_sec),
            cz
        ])
        vel_ff = np.array([
            self.radius * w * math.cos(w * t_sec),
            self.radius * w * math.cos(2.0 * w * t_sec),
            0.0
        ])
        return pos, vel_ff

    def _control_loop(self):
        current_pos = self._read_pose()
        if current_pos is None:
            return

        now = self.get_clock().now()
        dt  = max((now - self.prev_time).nanoseconds / 1e9, 0.001)

        if self.center is None:
            self.center = current_pos.copy()
            self.start_time = now
            self.get_logger().info("Centro registrado. Iniciando trayectoria.")

        t = (now - self.start_time).nanoseconds / 1e9
        target_pos, vel_ff = self._lemniscate(t)
        raw_error = target_pos - current_pos

        v_final     = np.zeros(3)
        vel_desired = np.zeros(3)
        vel_actual  = np.zeros(3)
        for i, cascade in enumerate(self._cascades):
            v_final[i], vel_desired[i], vel_actual[i] = cascade.compute(
                raw_error[i], current_pos[i], vel_ff[i], dt
            )

        v_norm = np.linalg.norm(v_final)
        if v_norm > self.max_speed:
            v_final = (v_final / v_norm) * self.max_speed
            v_norm  = self.max_speed

        self._publish_twist(v_final)

        self.history_data.append({
            'time':   t,
            'des_x':  target_pos[0],  'des_y':  target_pos[1],  'des_z':  target_pos[2],
            'act_x':  current_pos[0], 'act_y':  current_pos[1], 'act_z':  current_pos[2],
            'err_x':  raw_error[0],   'err_y':  raw_error[1],   'err_z':  raw_error[2],
            'vff_x':  vel_ff[0],      'vff_y':  vel_ff[1],
            'vdes_x': vel_desired[0], 'vdes_y': vel_desired[1],
            'vact_x': vel_actual[0],  'vact_y': vel_actual[1],
            'vel_mag': v_norm,
        })

        self.prev_time = now

    def _publish_twist(self, v_xyz: np.ndarray):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = "link_base"
        cmd.twist.linear.x  = float(v_xyz[0])
        cmd.twist.linear.y  = float(v_xyz[1])
        cmd.twist.linear.z  = float(v_xyz[2])
        self.servo_pub.publish(cmd)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.history_data:
            _p = pathlib.Path(__file__).resolve()
            out_dir = _p.parent
            for parent in _p.parents:
                if parent.name == 'install':
                    src = parent.parent / 'src' / 'xarm_ros2' / 'xarm_perturbations' / 'xarm_perturbations'
                    if src.exists():
                        out_dir = src
                    break

            df = pd.DataFrame(node.history_data)
            csv_path = out_dir / 'robot_evaluation.csv'
            df.to_csv(csv_path, index=False)
            node.get_logger().info(f"Datos guardados en '{csv_path}'")

            fig, axes = plt.subplots(1, 2, figsize=(14, 6))

            ax = axes[0]
            ax.plot(df['des_x'], df['des_y'], 'b--', lw=1.5, label='Deseada')
            ax.plot(df['act_x'], df['act_y'], 'r-',  lw=1.0, label='Real')
            ax.plot(df['des_x'].iloc[0], df['des_y'].iloc[0], 'go', ms=8, label='Inicio')
            ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
            ax.set_title('Trayectoria XY'); ax.legend(); ax.axis('equal'); ax.grid(True)

            ax2 = axes[1]
            t = df['time']
            ax2.plot(t, df['err_x'], label='err_x')
            ax2.plot(t, df['err_y'], label='err_y')
            ax2.plot(t, df['err_z'], label='err_z')
            ax2.set_xlabel('Tiempo [s]'); ax2.set_ylabel('Error [m]')
            ax2.set_title('Error de posición por eje'); ax2.legend(); ax2.grid(True)

            fig.tight_layout()
            plot_path = out_dir / 'robot_trajectory_xy.png'
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            node.get_logger().info(f"Grafica guardada en '{plot_path}'")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
