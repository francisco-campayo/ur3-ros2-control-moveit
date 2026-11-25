import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

class SmartTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('smart_trajectory_publisher')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            10)
            
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        # El orden exacto que espera nuestro controlador
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Lista de objetivos a visitar secuencialmente
        self.waypoints = [
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],  # Pose 1: Levantado
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]          # Pose 2: Reposo
        ]
        
        self.current_waypoint_index = 0
        self.target_pose = []
        self.is_moving = False
        
        # Tolerancia de error en radianes (~1 grado)
        self.tolerance = 0.02 
        
        # Arrancamos la secuencia tras 1 segundo
        self.timer = self.create_timer(1.0, self.start_sequence)

    def start_sequence(self):
        self.timer.cancel() # Solo queremos que se dispare una vez
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('¡Secuencia completa! Robot en destino final.')
            return

        self.target_pose = self.waypoints[self.current_waypoint_index]
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.target_pose
        point.time_from_start = Duration(sec=3, nanosec=0)

        msg.points = [point]
        
        self.get_logger().info(f'Enviando comando para waypoint {self.current_waypoint_index + 1}...')
        self.publisher_.publish(msg)
        self.is_moving = True

    def joint_state_callback(self, msg):
        if not self.is_moving:
            return
            
        # 1. Emparejamos el estado actual con el orden correcto de articulaciones
        current_positions = []
        for name in self.joint_names:
            try:
                idx = msg.name.index(name)
                current_positions.append(msg.position[idx])
            except ValueError:
                return # Si falta alguna articulación en el mensaje, ignoramos

        # 2. Calculamos el error máximo de todas las articulaciones
        max_error = 0.0
        for i in range(len(self.target_pose)):
            error = abs(self.target_pose[i] - current_positions[i])
            if error > max_error:
                max_error = error
                
        # 3. Comprobamos si hemos llegado
        if max_error < self.tolerance:
            self.get_logger().info(f'¡Waypoint {self.current_waypoint_index + 1} alcanzado con éxito!')
            self.is_moving = False
            self.current_waypoint_index += 1
            
            # Lanzamos el siguiente punto
            self.send_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = SmartTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()