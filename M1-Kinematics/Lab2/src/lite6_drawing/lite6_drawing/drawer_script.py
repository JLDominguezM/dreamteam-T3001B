
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import PoseStamped, Pose, Point
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import math
import numpy as np


from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TaskProgressColumn, TimeRemainingColumn
from rich.live import Live
from rich.panel import Panel

console = Console()

class Lite6Drawer(Node):

    def __init__(self):
        super().__init__('lite6_drawer')
        console.rule("[bold cyan]Lite6 Drawer Node[/bold cyan]")
        
        self.get_logger().set_level(LoggingSeverity.ERROR)
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[f"joint{i}" for i in range(1, 7)],
            base_link_name="link_base",
            end_effector_name="link_eef",
            
            group_name="lite6",
            callback_group=self.callback_group
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.marker_pub = self.create_publisher(Marker, 'drawing_trail', 10)
        self.stroke_id = 0 
        
        self.retract_dist = 0.03
        
        self.moveit2.max_velocity = 0.08  
        self.moveit2.max_acceleration = 0.05
        
        self.timer = self.create_timer(8.0, self.start_drawing)
        self.pose_published = False
        
    def get_current_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'link_base', 'link_eef', now, timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            current_pose = Pose()
            current_pose.position.x = trans.transform.translation.x
            current_pose.position.y = trans.transform.translation.y
            current_pose.position.z = trans.transform.translation.z
            current_pose.orientation.x = trans.transform.rotation.x
            current_pose.orientation.y = trans.transform.rotation.y
            current_pose.orientation.z = trans.transform.rotation.z
            current_pose.orientation.w = trans.transform.rotation.w
            
            return current_pose
        except TransformException as ex:
            console.print(f'[bold red]Error TF2: {ex}[/bold red]')
            return None

    def translate_pose_locally(self, base_pose, dx, dy, dz):
        """
        Traslación Inteligente Alineada a la Gravedad.
        dx: Ancho de la letra (Derecha)
        dy: Alto de la letra (Arriba)
        dz: Profundidad del plumón (Clavar/Despegar)
        """
        q = base_pose.orientation
        
        
        
        nx = 2.0*(q.x*q.z + q.y*q.w)
        ny = 2.0*(q.y*q.z - q.x*q.w)
        nz = 1.0 - 2.0*(q.x**2 + q.y**2)
        
        rx = ny
        ry = -nx
        rz = 0.0
        
        L = math.sqrt(rx**2 + ry**2)
        
        if L > 0.001: 
            
            rx = rx / L
            ry = ry / L
        else: 
            
            rx, ry, rz = 0.0, -1.0, 0.0 
            
        ux = ry * nz - rz * ny
        uy = rz * nx - rx * nz
        uz = rx * ny - ry * nx
        
        tx = (dx * rx) + (dy * ux) + (dz * nx)
        ty = (dx * ry) + (dy * uy) + (dz * ny)
        tz = (dx * rz) + (dy * uz) + (dz * nz)
        
        new_pose = Pose()
        new_pose.position.x = base_pose.position.x + tx
        new_pose.position.y = base_pose.position.y + ty
        new_pose.position.z = base_pose.position.z + tz
        
        new_pose.orientation = base_pose.orientation 
        
        return new_pose
    
    def get_letter_trajectory(self, letter):
        letters = {
            'A': [[0,0], [0.5,1], [1,0], [0.75,0.5], [0.25,0.5]],
            'B': [[0,0], [0,1], [0.8,1], [1,0.75], [0.8,0.5], [0,0.5], [0.8,0.5], [1,0.25], [0.8,0], [0,0]],
            'C': [[1,0.2], [0.8,0], [0.2,0], [0,0.2], [0,0.8], [0.2,1], [0.8,1], [1,0.8]],
            'D': [[0,0], [0,1], [0.5,1], [1,0.75], [1,0.25], [0.5,0], [0,0]],
            'E': [[1,0], [0,0], [0,0.5], [0.8,0.5], [0,0.5], [0,1], [1,1]],
            'F': [[0,0], [0,1], [1,1], [0,1], [0,0.5], [0.8,0.5]],
            'G': [[1,0.8], [0.8,1], [0.2,1], [0,0.8], [0,0.2], [0.2,0], [0.8,0], [1,0.2], [1,0.5], [0.5,0.5]],
            'H': [[0,1], [0,0], [0,0.5], [1,0.5], [1,1], [1,0]],
            'I': [[0.2,1], [0.8,1], [0.5,1], [0.5,0], [0.2,0], [0.8,0]],
            'J': [[0,0.5], [0.2,0], [0.8,0], [1,0.2], [1,1]],
            'K': [[0,1], [0,0], [0,0.5], [1,1], [0,0.5], [1,0]],
            'L': [[0,1], [0,0], [1,0]],
            'M': [[0,0], [0,1], [0.5,0.5], [1,1], [1,0]],
            'N': [[0,0], [0,1], [1,0], [1,1]],
            'O': [[0.5,0], [0,0.2], [0,0.8], [0.5,1], [1,0.8], [1,0.2], [0.5,0]],
            'P': [[0,0], [0,1], [0.8,1], [1,0.8], [1,0.7], [0.8,0.5], [0,0.5]],
            'Q': [[0.5,0], [0,0.2], [0,0.8], [0.5,1], [1,0.8], [1,0.2], [0.5,0], [0.8,0.2], [1,0]],
            'R': [[0,0], [0,1], [0.8,1], [1,0.8], [1,0.7], [0.8,0.5], [0,0.5], [0.5,0.5], [1,0]],
            'S': [[0,0.2], [0.2,0], [0.8,0], [1,0.2], [1,0.4], [0.8,0.5], [0.2,0.5], [0,0.6], [0,0.8], [0.2,1], [0.8,1], [1,0.8]],
            'T': [[0,1], [1,1], [0.5,1], [0.5,0]],
            'U': [[0,1], [0,0.2], [0.2,0], [0.8,0], [1,0.2], [1,1]],
            'V': [[0,1], [0.5,0], [1,1]],
            'W': [[0,1], [0.2,0], [0.5,0.5], [0.8,0], [1,1]],
            'X': [[0,1], [1,0], [0.5,0.5], [0,0], [1,1]],
            'Y': [[0,1], [0.5,0.5], [1,1], [0.5,0.5], [0.5,0]],
            'Z': [[0,1], [1,1], [0,0], [1,0]]
        }
        return letters.get(letter.upper(), [])
    
    def calculate_dynamic_size(self, word):
        max_width = 0.19
        max_height = 0.08 
        
        num_letters = len(word.replace(" ", ""))
        num_spaces = len(word) - 1
        total_width_units = num_letters + (num_spaces * 0.3)
        
        calculated_width = max_width / max(total_width_units, 1)
        final_height = min(calculated_width * 1.5, max_height)
        final_width = final_height * 0.65 
        spacing = final_width * 1.3
        
        console.print(f'[dim]Scaling: Letters of {final_height*100:.1f}cm in height[/dim]')
        return final_width, final_height, spacing

    def draw_letter(self, letter, letter_base_pose, w_size, h_size, progress_callback=None):
        trajectory = self.get_letter_trajectory(letter)
        if not trajectory: return False
        
        marker = Marker()
        marker.header.frame_id = "link_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "letters"
        marker.id = self.stroke_id
        self.stroke_id += 1 
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.003  
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0) 
        
        waypoints = []
        for point in trajectory:
            waypoint = self.translate_pose_locally(
                letter_base_pose, dx=(point[0] * w_size), dy=(point[1] * h_size), dz=0.0
            )
            waypoints.append(waypoint)
            
        try:
            total_points = len(waypoints)
            for i, waypoint in enumerate(waypoints):
                if progress_callback:
                    progress_callback(i + 1, total_points)
                    
                self.moveit2.move_to_pose(
                    position=[waypoint.position.x, waypoint.position.y, waypoint.position.z],
                    quat_xyzw=[waypoint.orientation.x, waypoint.orientation.y, waypoint.orientation.z, waypoint.orientation.w],
                    cartesian=True, cartesian_max_step=0.005, cartesian_fraction_threshold=0.95
                )
                self.moveit2.wait_until_executed()
                time.sleep(0.01)
                
                p = Point()
                p.x = waypoint.position.x
                p.y = waypoint.position.y
                p.z = waypoint.position.z
                marker.points.append(p)
                self.marker_pub.publish(marker)
                
            return True
        except Exception as e:
            console.print(f'[bold red]Error in Cartesian IK: {str(e)}[/bold red]')
            return False
    
    def draw_word(self, word, start_pose):
        console.rule(f"[bold yellow]Drawing Word: {word.upper()}[/bold yellow]")
        logging_severity = self.get_logger().get_effective_level()
        self.get_logger().set_level(LoggingSeverity.ERROR)
        
        w_size, h_size, letter_spacing = self.calculate_dynamic_size(word)
        current_offset = 0.0
        
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            BarColumn(bar_width=None),
            TaskProgressColumn(),
            TimeRemainingColumn(),
            console=console,
            transient=False 
        ) as progress:
            main_task = progress.add_task(f"[cyan]Drawing...", total=len(word))
            
            for i, letter in enumerate(word.upper()):
                if letter == ' ':
                    current_offset += letter_spacing
                    progress.advance(main_task)
                    continue
                    
                progress.update(main_task, description=f"[cyan]Drawing letter: [bold white]{letter}[/bold white]")
                
                letter_pose = self.translate_pose_locally(start_pose, dx=current_offset, dy=0.0, dz=0.0)
                hover_pose = self.translate_pose_locally(letter_pose, dx=0.0, dy=0.0, dz=-self.retract_dist)
                
                try:
                    self.moveit2.move_to_pose(
                        position=[hover_pose.position.x, hover_pose.position.y, hover_pose.position.z],
                        quat_xyzw=[hover_pose.orientation.x, hover_pose.orientation.y, hover_pose.orientation.z, hover_pose.orientation.w],
                        cartesian=True
                    )
                    self.moveit2.wait_until_executed()
                    
                    self.moveit2.move_to_pose(
                        position=[letter_pose.position.x, letter_pose.position.y, letter_pose.position.z],
                        quat_xyzw=[letter_pose.orientation.x, letter_pose.orientation.y, letter_pose.orientation.z, letter_pose.orientation.w],
                        cartesian=True
                    )
                    self.moveit2.wait_until_executed()

                    def update_letter_progress(current, total):
                         progress.update(main_task, description=f"[cyan]Drawing [bold white]{letter}[/bold white] (point {current}/{total})")

                    if not self.draw_letter(letter, letter_pose, w_size, h_size, progress_callback=update_letter_progress): 
                        self.get_logger().set_level(logging_severity)
                        return False
                    
                    end_pose = self.get_current_pose()
                    retreat_pose = self.translate_pose_locally(end_pose, dx=0.0, dy=0.0, dz=-self.retract_dist)
                    
                    self.moveit2.move_to_pose(
                        position=[retreat_pose.position.x, retreat_pose.position.y, retreat_pose.position.z],
                        quat_xyzw=[retreat_pose.orientation.x, retreat_pose.orientation.y, retreat_pose.orientation.z, retreat_pose.orientation.w],
                        cartesian=True
                    )
                    self.moveit2.wait_until_executed()
                    
                except Exception as e:
                    console.print(f"[bold red]Error during drawing: {e}[/bold red]")
                    self.get_logger().set_level(logging_severity) 
                    return False
                
                current_offset += letter_spacing
                progress.advance(main_task)
        
        self.get_logger().set_level(logging_severity) 
        console.print('[bold green]✔ Drawing sequence completed.[/bold green]')
        return True
        
    def start_drawing(self):
        if self.pose_published: return
        self.pose_published = True
        self.timer.cancel()
        
        target_word = "DREAMTEAM" 
        
        with console.status("[bold green]Acquiring initial pose for anchoring...", spinner="dots"):
             time.sleep(1.0)
             current_pose = self.get_current_pose()
        
        if current_pose is None:
            console.print("[bold red] Could not acquire initial pose.[/bold red]") 
            return
            
        try:
            self.draw_word(target_word, current_pose)
        except Exception as e:
            console.print(f'[bold red]Fatal error: {str(e)}[/bold red]')
            
    def pose_callback(self, msg): pass
        
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor
    drawer_node = Lite6Drawer()
    executor = MultiThreadedExecutor()
    executor.add_node(drawer_node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        drawer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()