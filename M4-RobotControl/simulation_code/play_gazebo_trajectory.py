import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
import math

class HyperSmoothPlayer(Node):
    def __init__(self):
        super().__init__('hyper_smooth_player')
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Poses extraídas de tu run_mujoco_simulation.py (en GRADOS)
        self.starting_pos = [-4.40, -92.24, 89.95, 55.11, 0.0, 0.0]
        self.zero_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        

        self.timer = self.create_timer(2.0, self.execute)
        self.done = False

    def get_s_curve_pos(self, t, duration, start, end):
        """Calcula la posición usando una rampa de aceleración suave."""
        fraction = t / duration
        smooth_f = 0.5 * (1 - math.cos(math.pi * fraction))
        return [math.radians(s + (e - s) * smooth_f) for s, e in zip(start, end)]

    def execute(self):
        if self.done: return
        self.get_logger().info('Iniciando trayectoria de alta fidelidad (100Hz)...')

        msg = JointTrajectory()
        msg.header = Header(frame_id='base')
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = self.joint_names

        hz = 100.0
        dt = 1.0 / hz
        move_time = 0.2
        pause_time = 0.05
        
        points = []
        steps = int(move_time * hz)

        for i in range(steps + 1):
            t = i * dt
            p = JointTrajectoryPoint()
            p.positions = self.get_s_curve_pos(t, move_time, self.starting_pos, self.zero_pos)
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            points.append(p)

        start_back = move_time + pause_time
        for i in range(steps + 1):
            t_relative = i * dt
            t_absolute = start_back + t_relative
            p = JointTrajectoryPoint()
            p.positions = self.get_s_curve_pos(t_relative, move_time, self.zero_pos, self.starting_pos)
            p.time_from_start = Duration(sec=int(t_absolute), nanosec=int((t_absolute % 1) * 1e9))
            points.append(p)

        msg.points = points
        self.publisher_.publish(msg)
        self.get_logger().info('¡Trayectoria fluida enviada con éxito!')
        self.done = True

def main(args=None):
    rclpy.init(args=args)
    node = HyperSmoothPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()