#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import pandas as pd

class SystemIdentifier(Node):
    def __init__(self):
        super().__init__("system_identifier")

        self.servo_pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parámetros del experimento de excitación
        self.dt = 0.02 # 50 Hz
        self.v_cmd_step = 0.05 # m/s (Velocidad segura pero excitable)
        
        # Tiempos de las fases
        self.t_idle = 1.0
        self.t_step = 1.5
        self.t_stop = 0.5
        
        # Variables de estado
        self.state = "INIT"
        self.start_time = None
        self.step_start_time = None
        self.data_log = []
        
        self.timer = self.create_timer(self.dt, self._experiment_loop)
        self.get_logger().info("🔬 Perfilador de Identificación de Planta Iniciado.")
        self.get_logger().info("⚠️ ADVERTENCIA: El robot se moverá en +X (5 cm aprox). Asegura el área.")

    def _read_x_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            return trans.transform.translation.x
        except Exception:
            return None

    def _publish_velocity(self, v_x):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "link_base"
        cmd.twist.linear.x = float(v_x)
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        self.servo_pub.publish(cmd)

    def _experiment_loop(self):
        current_x = self._read_x_pose()
        if current_x is None:
            return # Esperando a que TF esté listo

        now = self.get_clock().now()

        if self.state == "INIT":
            self.start_time = now
            self.state = "IDLE"
            self.get_logger().info("Fase 1: Reposo (1 segundo)...")

        t_elapsed = (now - self.start_time).nanoseconds / 1e9

        if self.state == "IDLE":
            self._publish_velocity(0.0)
            if t_elapsed >= self.t_idle:
                self.state = "STEP"
                self.step_start_time = t_elapsed
                self.get_logger().info("Fase 2: INYECTANDO ESCALÓN DE VELOCIDAD (1.5s)...")
                
        elif self.state == "STEP":
            self._publish_velocity(self.v_cmd_step)
            # Guardamos datos: tiempo relativo al inicio del escalón, y posición
            self.data_log.append({'t': t_elapsed - self.step_start_time, 'x': current_x})
            
            if (t_elapsed - self.step_start_time) >= self.t_step:
                self.state = "STOP"
                self.get_logger().info("Fase 3: Deteniendo el robot...")
                
        elif self.state == "STOP":
            self._publish_velocity(0.0)
            if (t_elapsed - self.step_start_time) >= (self.t_step + self.t_stop):
                self.state = "DONE"
                self.timer.cancel()
                self._analyze_system()

    def _analyze_system(self):
        self.get_logger().info("📊 Experimento finalizado. Procesando matemáticas...")
        
        df = pd.DataFrame(self.data_log)
        
        # 1. Derivada numérica para obtener velocidad real
        df['v_real'] = df['x'].diff() / df['t'].diff()
        df = df.dropna() # Quitar el primer NaN
        
        # 2. Encontrar Estado Estacionario (Vss)
        # Promediamos el último 30% del escalón donde ya debería estar estable
        t_stable_start = self.t_step * 0.7
        v_ss = df[df['t'] > t_stable_start]['v_real'].mean()
        
        # 3. Calcular Ganancia K
        K = v_ss / self.v_cmd_step
        
        # 4. Calcular Constante de Tiempo Tau (63.2% de Vss)
        v_target_tau = 0.632 * v_ss
        
        # Buscar el primer instante donde la velocidad supera el 63.2%
        df_tau = df[df['v_real'] >= v_target_tau]
        if not df_tau.empty:
            tau = df_tau.iloc[0]['t']
        else:
            tau = 0.0 # Error fallback
            self.get_logger().error("No se pudo calcular Tau. Datos muy ruidosos.")

        # Guardar CSV por si queremos graficar en Python/Excel
        df.to_csv('step_response_data.csv', index=False)
        
        self.get_logger().info("=====================================================")
        self.get_logger().info("🏆 RESULTADOS DE IDENTIFICACIÓN DE LA PLANTA 🏆")
        self.get_logger().info(f"Velocidad en Estado Estacionario (Vss): {v_ss:.4f} m/s")
        self.get_logger().info(f"Ganancia del Sistema (K): {K:.4f}")
        self.get_logger().info(f"Constante de Tiempo (Tau): {tau:.4f} segundos")
        self.get_logger().info("=====================================================")
        self.get_logger().info("Pasa estos valores K y Tau para calcular tus ganancias finales.")
        
        # Matar el nodo limpiamente
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = SystemIdentifier()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("system_identifier").info("Cerrando nodo.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()