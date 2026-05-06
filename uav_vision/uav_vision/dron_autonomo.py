import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import time

class DronExploradorReactivo(Node):
    def __init__(self):
        super().__init__('dron_autonomo')
        self.publisher_ = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/uav/odom', self.odom_callback, 10)
        self.verify_sub = self.create_subscription(Point, '/uav/verify_point', self.verify_callback, 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_pose = None
        self.verification_point = None
        self.start_time = time.time()
        
        # Parámetros tácticos
        self.limit_x, self.limit_y = 25.0, 25.0
        self.patrol_speed = 1.5 # Patrulla rápida
        
        self.get_logger().info('--- EXPLORADOR REACTIVO VELOZ LISTO ---')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def verify_callback(self, msg):
        # Recibir punto de interés del detector
        self.verification_point = msg
        # Resetear temporizador de "abandono" si sigue viendo el objeto
        self.last_verify_time = time.time()

    def timer_callback(self):
        if self.current_pose is None: return
        
        elapsed = time.time() - self.start_time
        msg = Twist()
        x, y, z = self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z

        # 1. LÓGICA DE NAVEGACIÓN
        if self.verification_point:
            # Ir al punto de interés para verificar
            dx = self.verification_point.x - x
            dy = self.verification_point.y - y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist > 0.3:
                self.get_logger().info(f'🔎 Verificando sospecha a {dist:.1f}m...', once=True)
                angle_to_point = math.atan2(dy, dx)
                
                # Obtener yaw actual
                q = self.current_pose.orientation
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                error_ang = angle_to_point - yaw
                while error_ang > math.pi: error_ang -= 2 * math.pi
                while error_ang < -math.pi: error_ang += 2 * math.pi

                msg.linear.x = 0.8 # Aproximación veloz
                msg.angular.z = 1.0 * error_ang
            else:
                self.get_logger().info('✅ Punto alcanzado.')
                msg.linear.x = 0.0
                msg.angular.z = 0.6 # Giro rápido para verificar
                
            # Si pasa tiempo sin recibir nuevas detecciones, volver a patrulla
            if time.time() - self.last_verify_time > 3.0:
                self.verification_point = None
        
        elif abs(x) > self.limit_x or abs(y) > self.limit_y:
            # Regresar al centro rápido
            msg.linear.x = 0.8
            msg.angular.z = 1.0 * math.atan2(-y, -x)
        else:
            # Patrulla veloz
            msg.linear.x = self.patrol_speed
            msg.angular.z = 0.4 * math.sin(elapsed * 0.4)

        # 2. Mantenimiento de altura a 0.8m
        if z < 0.75: msg.linear.z = 0.1
        elif z > 0.85: msg.linear.z = -0.1
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DronExploradorReactivo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
