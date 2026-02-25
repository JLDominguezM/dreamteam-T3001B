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

class FFPDAxis:
    def __init__(self, kp: float, kd: float,
                 deadband: float = 0.002,
                 alpha_d: float = 0.08,
                 max_slew: float = 0.003,
                 alpha_out: float = 0.10):
        self.kp        = kp
        self.kd        = kd
        self.deadband  = deadband
        self.alpha_d   = alpha_d     # filtro derivada (0.08 = pesado)
        self.max_slew  = max_slew    # máx cambio por tick
        self.alpha_out = alpha_out   # coef para cada etapa del doble EMA

        self._prev_error = 0.0
        self._d_filt     = 0.0
        self._prev_raw   = 0.0
        self._ema1       = 0.0       # primera etapa EMA
        self._ema2       = 0.0       # segunda etapa EMA

    def reset(self):
        self._prev_error = 0.0
        self._d_filt     = 0.0
        self._prev_raw   = 0.0
        self._ema1       = 0.0
        self._ema2       = 0.0

    def compute(self, pos_error: float, vel_ff: float, dt: float) -> float:
        # Deadband
        if abs(pos_error) < self.deadband:
            error = 0.0
        else:
            error = pos_error - math.copysign(self.deadband, pos_error)

        # P
        p = self.kp * error

        # D filtrado (EMA pesado en la derivada = anti-ruido)
        raw_d       = (error - self._prev_error) / dt
        self._d_filt = self.alpha_d * raw_d + (1.0 - self.alpha_d) * self._d_filt
        d = self.kd * self._d_filt

        self._prev_error = error

        # Comando crudo = feedforward + corrección PD
        raw_cmd = vel_ff + p + d

        # Rate limiter
        delta = raw_cmd - self._prev_raw
        if abs(delta) > self.max_slew:
            raw_cmd = self._prev_raw + math.copysign(self.max_slew, delta)
        self._prev_raw = raw_cmd

        # Doble EMA (filtro segundo orden): elimina ruido sin exceso de lag
        a = self.alpha_out
        self._ema1 = a * raw_cmd   + (1.0 - a) * self._ema1
        self._ema2 = a * self._ema1 + (1.0 - a) * self._ema2

        return self._ema2

class PositionController(Node):
    def __init__(self):
        super().__init__("position_controller")

        self.declare_parameter('output_topic', '/servo_server/delta_twist_cmds')

        self.declare_parameter('kp_x', 2.2)
        self.declare_parameter('kp_y', 1.8)
        self.declare_parameter('kp_z', 0.8)

        self.declare_parameter('kd_x', 0.2)
        self.declare_parameter('kd_y', 0.2)
        self.declare_parameter('kd_z', 0.2)

        self.declare_parameter('max_speed', 0.30)
        self.declare_parameter('deadband',  0.002)
        self.declare_parameter('radius',    0.06)
        self.declare_parameter('frequency', 0.06)
        self.declare_parameter('soft_start_duration', 3.0)  # segundos de rampa

        output_topic   = self.get_parameter('output_topic').value
        self.max_speed = float(self.get_parameter('max_speed').value)
        deadband       = float(self.get_parameter('deadband').value)
        self.radius    = float(self.get_parameter('radius').value)
        self.frequency = float(self.get_parameter('frequency').value)
        self.soft_start_dur = float(self.get_parameter('soft_start_duration').value)

        kp = [self.get_parameter(f'kp_{a}').value for a in ('x', 'y', 'z')]
        kd = [self.get_parameter(f'kd_{a}').value for a in ('x', 'y', 'z')]

        self._axes = [
            FFPDAxis(kp[i], kd[i], deadband)
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
        self.get_logger().info(f"FF+PD iniciado | Kp={kp} Kd={kd}")

    def _read_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            return np.array([t.transform.translation.x,
                             t.transform.translation.y,
                             t.transform.translation.z])
        except Exception:
            return None

    def _lemniscate(self, t_sec: float) -> tuple[np.ndarray, np.ndarray]:
        cx, cy, cz = self.center
        w = 2.0 * math.pi * self.frequency

        # Soft start: rampa suave de 0→1 usando coseno (sin discontinuidad)
        T = self.soft_start_dur
        if t_sec < T:
            s    = 0.5 * (1.0 - math.cos(math.pi * t_sec / T))  # 0→1 suave
            ds   = 0.5 * math.pi / T * math.sin(math.pi * t_sec / T)
        else:
            s    = 1.0
            ds   = 0.0

        x_rel  = self.radius * math.sin(w * t_sec)
        y_rel  = self.radius * math.sin(w * t_sec) * math.cos(w * t_sec)
        vx_raw = self.radius * w * math.cos(w * t_sec)
        vy_raw = self.radius * w * math.cos(2.0 * w * t_sec)

        pos = np.array([cx + s * x_rel, cy + s * y_rel, cz])
        # d/dt(s * f(t)) = ds * f(t) + s * f'(t)
        vel_ff = np.array([
            ds * x_rel + s * vx_raw,
            ds * y_rel + s * vy_raw,
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

        v_final = np.zeros(3)
        for i, axis in enumerate(self._axes):
            v_final[i] = axis.compute(raw_error[i], vel_ff[i], dt)

        v_norm = np.linalg.norm(v_final)
        if v_norm > self.max_speed:
            v_final = (v_final / v_norm) * self.max_speed
            v_norm  = self.max_speed

        self._publish_twist(v_final)

        self.history_data.append({
            'time':  t,
            'des_x': target_pos[0],  'des_y': target_pos[1],  'des_z': target_pos[2],
            'act_x': current_pos[0], 'act_y': current_pos[1], 'act_z': current_pos[2],
            'err_x': raw_error[0],   'err_y': raw_error[1],   'err_z': raw_error[2],
            'cmd_x': v_final[0],     'cmd_y': v_final[1],     'cmd_z': v_final[2],
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

            # ── Métricas cuantitativas ──────────────────────────────────
            t   = df['time'].to_numpy()
            ex  = df['err_x'].to_numpy()
            ey  = df['err_y'].to_numpy()
            ez  = df['err_z'].to_numpy()
            e_total = np.sqrt(ex**2 + ey**2 + ez**2)

            rmse_x   = float(np.sqrt(np.mean(ex**2)))
            rmse_y   = float(np.sqrt(np.mean(ey**2)))
            rmse_z   = float(np.sqrt(np.mean(ez**2)))
            rmse_tot = float(np.sqrt(np.mean(e_total**2)))
            max_err  = float(np.max(np.abs(e_total)))

            node.get_logger().info(
                f"RMSE_x={rmse_x:.5f} m | RMSE_y={rmse_y:.5f} m | "
                f"RMSE_z={rmse_z:.5f} m | RMSE_total={rmse_tot:.5f} m | "
                f"Max_error={max_err:.5f} m"
            )

            metrics_path = out_dir / 'robot_metrics.txt'
            with open(metrics_path, 'w') as f:
                f.write(f"RMSE_x:     {rmse_x:.6f} m\n")
                f.write(f"RMSE_y:     {rmse_y:.6f} m\n")
                f.write(f"RMSE_z:     {rmse_z:.6f} m\n")
                f.write(f"RMSE_total: {rmse_tot:.6f} m\n")
                f.write(f"Max_error:  {max_err:.6f} m\n")
                f.write(f"Samples:    {len(df)}\n")
                f.write(f"Duration:   {t[-1]:.2f} s\n")

            # ── Gráfica 1: Trayectoria XY ──────────────────────────────
            fig1, ax1 = plt.subplots(figsize=(8, 7))
            ax1.plot(df['des_x'].to_numpy(), df['des_y'].to_numpy(),
                     'b--', lw=1.5, label='Deseada')
            ax1.plot(df['act_x'].to_numpy(), df['act_y'].to_numpy(),
                     'r-', lw=1.0, label='Real')
            ax1.plot(df['des_x'].iloc[0], df['des_y'].iloc[0],
                     'go', ms=8, label='Inicio')
            ax1.set_xlabel('X [m]'); ax1.set_ylabel('Y [m]')
            ax1.set_title('Trayectoria XY — Lemniscata deseada vs real')
            ax1.legend(); ax1.axis('equal'); ax1.grid(True)
            fig1.tight_layout()
            fig1.savefig(out_dir / 'plot_trajectory_xy.png', dpi=150)
            plt.close(fig1)

            # ── Gráfica 2: Desired vs Actual por eje ───────────────────
            fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
            for i, (axis_label, dcol, acol) in enumerate(
                [('X', 'des_x', 'act_x'),
                 ('Y', 'des_y', 'act_y'),
                 ('Z', 'des_z', 'act_z')]):
                axes2[i].plot(t, df[dcol].to_numpy(), 'b--', lw=1.2, label=f'Deseada {axis_label}')
                axes2[i].plot(t, df[acol].to_numpy(), 'r-',  lw=0.8, label=f'Real {axis_label}')
                axes2[i].set_ylabel(f'{axis_label} [m]')
                axes2[i].legend(loc='upper right'); axes2[i].grid(True)
            axes2[2].set_xlabel('Tiempo [s]')
            axes2[0].set_title('Posición deseada vs real por eje')
            fig2.tight_layout()
            fig2.savefig(out_dir / 'plot_desired_vs_actual.png', dpi=150)
            plt.close(fig2)

            # ── Gráfica 3: Error por eje ───────────────────────────────
            fig3, ax3 = plt.subplots(figsize=(12, 5))
            ax3.plot(t, ex * 1000, label='err_x')
            ax3.plot(t, ey * 1000, label='err_y')
            ax3.plot(t, ez * 1000, label='err_z')
            ax3.plot(t, e_total * 1000, 'k--', lw=0.8, label='||error||')
            ax3.set_xlabel('Tiempo [s]'); ax3.set_ylabel('Error [mm]')
            ax3.set_title(f'Error de posición  —  RMSE_total={rmse_tot*1000:.2f} mm, Max={max_err*1000:.2f} mm')
            ax3.legend(); ax3.grid(True)
            fig3.tight_layout()
            fig3.savefig(out_dir / 'plot_error.png', dpi=150)
            plt.close(fig3)

            # ── Gráfica 4: Velocidad comandada ─────────────────────────
            fig4, ax4 = plt.subplots(figsize=(12, 5))
            ax4.plot(t, df['cmd_x'].to_numpy(), label='cmd_x')
            ax4.plot(t, df['cmd_y'].to_numpy(), label='cmd_y')
            ax4.plot(t, df['cmd_z'].to_numpy(), label='cmd_z')
            ax4.plot(t, df['vel_mag'].to_numpy(), 'k--', lw=0.8, label='||vel||')
            ax4.set_xlabel('Tiempo [s]'); ax4.set_ylabel('Velocidad [m/s]')
            ax4.set_title('Velocidad comandada por eje')
            ax4.legend(); ax4.grid(True)
            fig4.tight_layout()
            fig4.savefig(out_dir / 'plot_velocity.png', dpi=150)
            plt.close(fig4)

            node.get_logger().info(f"Graficas guardadas en '{out_dir}'")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
