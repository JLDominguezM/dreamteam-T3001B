#!/usr/bin/env python3
import math
import ast
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

class PositionController(Node):
    def __init__(self):
        super().__init__("position_controller")

        self.declare_parameter('output_topic', '/servo_server/delta_twist_cmds')
        
        # --- GANANCIAS HÍBRIDAS DEFINITIVAS ---
        # base
        # self.declare_parameter('kp', '[32.18, 32.18, 0.0]')
        # self.declare_parameter('kd', '[3.42, 3.42, 0.0]')
        # self.declare_parameter('ki', '[0.0, 0.0, 0.0]')

        # Sine
        # self.declare_parameter('kp', '[130.315, 100.18, 10.0]')
        # self.declare_parameter('kd', '[8.034, 7.42, 6.0]')
        # self.declare_parameter('ki', '[0.0, 0.0, 0.0]')

        self.declare_parameter('kp', '[130.315, 100.18, 10.0]')
        self.declare_parameter('kd', '[8.034, 7.42, 6.0]')
        self.declare_parameter('ki', '[0.0, 0.0, 0.0]')

        self.declare_parameter('max_speed', 0.10)
        self.declare_parameter('deadband', 0.002)
        
        self.declare_parameter('radius', 0.06)
        self.declare_parameter('frequency', 0.06)

        output_topic = self.get_parameter('output_topic').value
        self.kp = np.array(ast.literal_eval(self.get_parameter('kp').value), dtype=float)
        self.kd = np.array(ast.literal_eval(self.get_parameter('kd').value), dtype=float)
        self.ki = np.array(ast.literal_eval(self.get_parameter('ki').value), dtype=float)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.radius = float(self.get_parameter('radius').value)
        self.frequency = float(self.get_parameter('frequency').value)

        self.servo_pub = self.create_publisher(TwistStamped, output_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Variables
        self.prev_error = np.zeros(3)
        self.integral_error = np.zeros(3)
        self.prev_d_error_filtered = np.zeros(3)
        self.history_data = []
        
        self.center = None
        self.start_time = self.get_clock().now()
        self.prev_time = self.get_clock().now()

        self.timer = self.create_timer(0.02, self._control_loop)
        self.get_logger().info(f"🦸 Multiverse Savior PID Started | Kp: {self.kp}")

    def _read_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            return np.array([
                trans.transform.translation.x, 
                trans.transform.translation.y, 
                trans.transform.translation.z
            ])
        except Exception:
            return None

    def _custom_target(self, t_sec: float):
        cx, cy, cz = self.center
        w = 2.0 * math.pi * self.frequency
        
        # Nueva ecuación Lemniscata: Arranca EXACTAMENTE en (0,0). 
        # Cero fase inicial, seguimiento perfecto garantizado.
        x_rel = self.radius * math.sin(w * t_sec)
        y_rel = self.radius * math.sin(w * t_sec) * math.cos(w * t_sec)
        
        return np.array([cx + x_rel, cy + y_rel, cz])

    def _control_loop(self):
        current_pos = self._read_pose()
        if current_pos is None:
            return

        now = self.get_clock().now()
        dt = max((now - self.prev_time).nanoseconds / 1e9, 0.001)

        if self.center is None:
            self.center = current_pos.copy()
            self.get_logger().info("Centro registrado. Iniciando trayectoria perfecta.")
            self.start_time = now

        t = (now - self.start_time).nanoseconds / 1e9
        target_pos = self._custom_target(t)

        # 1. Error directo y Deadband continuo
        raw_error = target_pos - current_pos
        error = np.where(np.abs(raw_error) < self.deadband, 0.0, raw_error - np.sign(raw_error) * self.deadband)

        p_term = self.kp * error

        # 2. Derivada con filtro
        raw_d_error = (error - self.prev_error) / dt
        alpha_d = 0.2 
        d_error_filtered = alpha_d * raw_d_error + (1 - alpha_d) * self.prev_d_error_filtered
        d_term = self.kd * d_error_filtered

        # 3. Integración condicionada (desactivada por Ki=0, pero lista por si acaso)
        v_preliminary = p_term + d_term + (self.ki * self.integral_error)
        for i in range(3):
            if abs(error[i]) > 0.0:
                if abs(v_preliminary[i]) < self.max_speed or (np.sign(error[i]) != np.sign(v_preliminary[i])):
                    self.integral_error[i] += error[i] * dt
        
        self.integral_error = np.clip(self.integral_error, -0.015, 0.015)
        i_term = self.ki * self.integral_error

        # 4. Cálculo final y Saturación correcta
        v_final = p_term + i_term + d_term
        
        v_norm_commanded = np.linalg.norm(v_final)
        if v_norm_commanded > self.max_speed:
            v_final = (v_final / v_norm_commanded) * self.max_speed
            v_norm_commanded = self.max_speed # CORRECCIÓN: Guardar el valor real comandado

        self._publish_twist(v_final)

        # 5. Registro de Evaluación Preciso
        self.history_data.append({
            'time': t,
            'des_x': target_pos[0], 'des_y': target_pos[1], 'des_z': target_pos[2],
            'act_x': current_pos[0], 'act_y': current_pos[1], 'act_z': current_pos[2],
            'err_x': raw_error[0], 'err_y': raw_error[1], 'err_z': raw_error[2],
            'vel_mag': v_norm_commanded
        })

        self.prev_error = error
        self.prev_d_error_filtered = d_error_filtered
        self.prev_time = now

    def _publish_twist(self, v_xyz: np.ndarray):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "link_base"
        cmd.twist.linear.x = float(v_xyz[0])
        cmd.twist.linear.y = float(v_xyz[1])
        cmd.twist.linear.z = float(v_xyz[2])
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
            out_dir = pathlib.Path(__file__).parent
            csv_path = out_dir / 'robot_evaluation.csv'
            df = pd.DataFrame(node.history_data)
            df.to_csv(csv_path, index=False)
            node.get_logger().info(f"📊 Datos guardados en '{csv_path}'")

            # --- Gráfica trayectoria XY: lemniscata deseada vs real ---
            fig, ax = plt.subplots(figsize=(8, 6))
            ax.plot(df['des_x'].to_numpy(), df['des_y'].to_numpy(), 'b--', linewidth=1.5, label='Deseada (lemniscata)')
            ax.plot(df['act_x'].to_numpy(), df['act_y'].to_numpy(), 'r-',  linewidth=1.0, label='Real (robot)')
            ax.plot(df['des_x'].iloc[0], df['des_y'].iloc[0], 'go', markersize=8, label='Inicio')
            ax.set_xlabel('X [m]')
            ax.set_ylabel('Y [m]')
            ax.set_title('Trayectoria XY — Lemniscata deseada vs real')
            ax.legend()
            ax.axis('equal')
            ax.grid(True)
            fig.tight_layout()
            plot_path = out_dir / 'robot_trajectory_xy.png'
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            node.get_logger().info(f"📈 Gráfica guardada en '{plot_path}'")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()