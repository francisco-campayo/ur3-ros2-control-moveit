import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

from my_ur_control.interpolators import QuinticInterpolator

class KinematicController(Node):
    def __init__(self):
        super().__init__('kinematic_controller')

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.num_joints = len(self.joint_names)

        self.vel_pub = self.create_publisher(
            Float64MultiArray, 
            '/forward_velocity_controller/commands', 
            10)

        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10)

        self.current_q = [0.0] * self.num_joints
        self.is_ready = False

        self.Kp = 1.8 
        self.duration = 3.0
        
        # Secuencia de movimientos
        self.waypoints = [
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],  # Pose 1: Levantado
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]          # Pose 2: Reposo (Estirado)
        ]
        self.current_wp_idx = 0
        
        self.interpolator = None
        self.start_time = 0.0

        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("Controlador cinemático listo. Esperando estado inicial...")

    def state_callback(self, msg):
        try:
            for i, name in enumerate(self.joint_names):
                idx = msg.name.index(name)
                self.current_q[i] = msg.position[idx]
            
            # Inicializamos el primer movimiento usando la pose real de Gazebo como inicio
            if not self.is_ready:
                self.interpolator = QuinticInterpolator(self.current_q, self.waypoints[0], self.duration)
                self.start_time = time.time()
                self.is_ready = True
                self.get_logger().info("¡Datos recibidos! Iniciando waypoint 1...")
        except ValueError:
            pass

    # MÁQUINA DE ESTADOS
    def control_loop(self):
        if not self.is_ready:
            return

        t = time.time() - self.start_time

        # 1. Obtenemos las referencias matemáticas
        # Si 't' supera los 3 segundos, nuestro interpolador quíntico está programado 
        # para devolver inteligentemente la posición final y velocidad 0.0.
        q_ref, dq_ref = self.interpolator.evaluate(t)

        # 2. Calculamos el error de cada articulación
        errores = [q_ref[i] - self.current_q[i] for i in range(self.num_joints)]
        error_maximo = max([abs(e) for e in errores])

        # 3. NUEVA LÓGICA DE TRANSICIÓN: Tiempo superado Y error estabilizado
        if t > self.duration and error_maximo < 0.01:
            self.current_wp_idx += 1
            
            if self.current_wp_idx < len(self.waypoints):
                self.get_logger().info(f"Estabilizado. Iniciando waypoint {self.current_wp_idx + 1}...")
                start_pose = self.waypoints[self.current_wp_idx - 1]
                end_pose = self.waypoints[self.current_wp_idx]
                
                self.interpolator = QuinticInterpolator(start_pose, end_pose, self.duration)
                self.start_time = time.time()
                return # Salimos en este ciclo para reiniciar los cálculos limpios en el siguiente
            else:
                self.get_logger().info("Secuencia completa y posición final estabilizada.")
                vel_cmd = Float64MultiArray()
                vel_cmd.data = [0.0] * self.num_joints
                self.vel_pub.publish(vel_cmd)
                self.timer.cancel()
                return

        # 4. Aplicamos la ley de control cinemático (Feedforward + Proporcional)
        vel_cmd = Float64MultiArray()
        vel_cmd.data = [0.0] * self.num_joints
        
        for i in range(self.num_joints):
            vel_cmd.data[i] = dq_ref[i] + (self.Kp * errores[i])

        # 5. Enviamos el vector de velocidades
        self.vel_pub.publish(vel_cmd)

    # BUCLE INFINITO
    # def control_loop(self):
    #     if not self.is_ready:
    #         return

    #     t = time.time() - self.start_time

    #     # 1. Obtenemos las referencias matemáticas
    #     q_ref, dq_ref = self.interpolator.evaluate(t)

    #     # 2. Calculamos el error máximo
    #     errores = [q_ref[i] - self.current_q[i] for i in range(self.num_joints)]
    #     error_maximo = max([abs(e) for e in errores])

    #     # 3. LÓGICA DE BUCLE INFINITO
    #     if t > self.duration and error_maximo < 0.01:
    #         # Avanzamos al siguiente índice, y si llegamos al final, volvemos al 0
    #         self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            
    #         self.get_logger().info(f"Estabilizado. Viajando al waypoint {self.current_wp_idx + 1} en bucle...")
            
    #         # El truco del -1: si el índice es 0, toma el último punto visitado como origen
    #         start_pose = self.waypoints[self.current_wp_idx - 1]
    #         end_pose = self.waypoints[self.current_wp_idx]
            
    #         self.interpolator = QuinticInterpolator(start_pose, end_pose, self.duration)
    #         self.start_time = time.time()
    #         return # Salimos para iniciar el nuevo ciclo limpio

    #     # 4. Aplicamos la ley de control cinemático
    #     vel_cmd = Float64MultiArray()
    #     vel_cmd.data = [0.0] * self.num_joints
        
    #     for i in range(self.num_joints):
    #         vel_cmd.data[i] = dq_ref[i] + (self.Kp * errores[i])

    #     # 5. Enviamos velocidades
    #     self.vel_pub.publish(vel_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()