#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from pynput import keyboard
from enum import Enum

class RobotState(Enum):
    RUNNING = 1
    PAUSED = 2
    HOME = 3

class CustomMakerXArmLite6(Node):
    """
    Controlador de trayectoria personalizada para xArm Lite 6.
    Implementa trayectoria de Lemniscata (Infinito) y controlador PID con Anti-windup.
    """

    def __init__(self):
        super().__init__("custom_maker_xarm_lite6")

        # --- Máquina de Estados ---
        self.robot_state = RobotState.RUNNING

        # --- Publicador y TF ---
        self.servo_pub = self.create_publisher(TwistStamped, "/servo_server/delta_twist_cmds", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Parámetros de Trayectoria (Task A) ---
        self.radius = 0.08          # Amplitud de la trayectoria
        self.frequency = 0.06       # Velocidad de ejecución
        self.plane = "xy"           
        
        # --- Parámetros del Controlador (Task B) ---
        
        # self.kp = np.array([2.0, 2.0, 2.0])
        # self.ki = np.array([0.0, 0.0, 0.0])  # Ganancia Integral para Bonus
        # self.kd = np.array([0.6, 0.6, 0.6])
        
        # para normal y senoidal
        # self.kp = np.array([2.0, 2.0, 2.0])
        # self.ki = np.array([0.0, 0.0, 0.0])  # Ganancia Integral para Bonus
        # self.kd = np.array([0.08, 0.08, 0.08])
        
        
        self.kp = np.array([1.2, 1.2, 1.2])
        self.ki = np.array([0.01, 0.01, 0.01])  # Un toque para el bonus y corregir el drift
        self.kd = np.array([0.02, 0.02, 0.02])  # Mucho más damping para el ruido

        # Anti-windup y Seguridad
        self.integral_error = np.zeros(3)
        self.integral_limit = 0.05           # Límite para evitar acumulación excesiva
        self.epsilon = np.array([0.002, 0.002, 0.002])  # Deadband: 2mm
        self.max_speed = 0.10                # Saturación: 0.10 m/s

        # Home y Memoria
        self.home_position = np.array([0.227, 0.00, 0.468])
        self.center = None
        self.start_time = self.get_clock().now()
        self.prev_error = np.zeros(3)
        self.prev_time = self.get_clock().now()
        self.last_info_time = self.get_clock().now()

        self._start_keyboard()
        self.timer = self.create_timer(0.02, self._loop) # 50 Hz

        self.get_logger().info("CustomMaker Initialized. Lemniscate Mode ACTIVE.")

    def _start_keyboard(self):
        def on_press(key):
            if hasattr(key, "char") and key.char == "p":
                if self.robot_state == RobotState.RUNNING:
                    self.robot_state = RobotState.PAUSED
                    self._publish_zero()
                elif self.robot_state == RobotState.PAUSED:
                    self.robot_state = RobotState.RUNNING
                    self.prev_time = self.get_clock().now()
            if hasattr(key, "char") and key.char == "h":
                self.robot_state = RobotState.HOME

        self.keyboard_listener = keyboard.Listener(on_press=on_press)
        self.keyboard_listener.start()

    def _read_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            return np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        except Exception:
            return None

    # --- Task A: Generador Matemático (Lemniscata) ---
    def _custom_target(self, t_sec: float):
        cx, cy, cz = self.center
        a = self.radius
        w = 2.0 * math.pi * self.frequency
        
        # Soft-start ramp (5 pts de rúbrica)
        ramp = min(max(t_sec / 2.0, 0.0), 1.0)
        curr_a = ramp * a

        # Ecuación: Lemniscata de Bernoulli
        denom = 1 + math.sin(w * t_sec)**2
        x_rel = (curr_a * math.cos(w * t_sec)) / denom
        y_rel = (curr_a * math.sin(w * t_sec) * math.cos(w * t_sec)) / denom

        return np.array([cx + x_rel, cy + y_rel, cz], dtype=float)

    def _publish_twist(self, v_xyz: np.ndarray):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "link_base"
        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z = map(float, v_xyz)
        self.servo_pub.publish(cmd)

    def _publish_zero(self):
        self._publish_twist(np.zeros(3))

    # --- Task B: Controlador PID con Anti-windup ---
    def _servo_to(self, target_pos: np.ndarray):
        current = self._read_pose()
        if current is None: return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0: dt = 1e-6

        error = target_pos - current
        
        # Lógica Integral con Anti-windup (Bonus 5 pts)
        self.integral_error += error * dt
        self.integral_error = np.clip(self.integral_error, -self.integral_limit, self.integral_limit)

        d_error = (error - self.prev_error) / dt

        # Ley de control PID
        v = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * d_error)

        # Deadband y Saturación (Requisitos Task B)
        active_mask = np.abs(error) > self.epsilon
        v = np.where(active_mask, v, 0.0)
        v = np.clip(v, -self.max_speed, self.max_speed)

        self._publish_twist(v)
        self.prev_error, self.prev_time = error, now

    def _loop(self):
        if self.center is None:
            p = self._read_pose()
            if p is not None:
                self.center = p.copy()
                self.get_logger().info(f"✅ Center set to {self.center.round(3)}")
            return

        if self.robot_state == RobotState.PAUSED: return

        if self.robot_state == RobotState.HOME:
            current = self._read_pose()
            if current is None: return
            error = self.home_position - current
            if np.linalg.norm(error) < 0.005:
                self.robot_state = RobotState.RUNNING
                self.center = current.copy()
                return
            v = np.clip(np.sign(error) * 0.05, -self.max_speed, self.max_speed)
            self._publish_twist(v)
            return

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        target = self._custom_target(t) # Llamada corregida
        self._servo_to(target)

def main(args=None):
    rclpy.init(args=args)
    node = CustomMakerXArmLite6()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()