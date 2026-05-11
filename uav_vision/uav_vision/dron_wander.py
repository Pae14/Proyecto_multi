import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import time
import random

class DronWander(Node):
    def __init__(self):
        super().__init__('dron_wander')
        self.publisher_ = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/uav/odom', self.odom_callback, 10)
        self.verify_sub = self.create_subscription(Point, '/uav/verify_point', self.verify_callback, 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_pose = None
        self.verification_point = None
        self.last_verify_time = time.time()
        
        # Parámetros Wander
        self.target_x = 0.0
        self.target_y = 0.0
        self.limit_x, self.limit_y = 20.0, 20.0
        self.speed = 1.2
        self.new_target()
        
        self.get_logger().info('--- NAVEGACIÓN WANDER (ERRANTE) INICIADA ---')

    def new_target(self):
        self.target_x = random.uniform(-self.limit_x, self.limit_x)
        self.target_y = random.uniform(-self.limit_y, self.limit_y)
        self.get_logger().info(f'📍 Nuevo destino wander: [{self.target_x:.1f}, {self.target_y:.1f}]')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def verify_callback(self, msg):
        self.verification_point = msg
        self.last_verify_time = time.time()

    def timer_callback(self):
        if self.current_pose is None: return
        
        msg = Twist()
        x, y, z = self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z

        # 1. LÓGICA DE PRIORIDAD
        if self.verification_point:
            # PRIORIDAD: Ir al punto de interés del detector
            dx = self.verification_point.x - x
            dy = self.verification_point.y - y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist > 0.4:
                angle_to_point = math.atan2(dy, dx)
                q = self.current_pose.orientation
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                error_ang = angle_to_point - yaw
                while error_ang > math.pi: error_ang -= 2 * math.pi
                while error_ang < -math.pi: error_ang += 2 * math.pi

                msg.linear.x = 1.0 
                msg.angular.z = 1.2 * error_ang
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.8 # Giro de inspección
                
            if time.time() - self.last_verify_time > 4.0:
                self.verification_point = None
                self.new_target()
        
        else:
            # WANDER: Ir al objetivo aleatorio
            dx = self.target_x - x
            dy = self.target_y - y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 1.0:
                self.new_target()
            
            angle_to_target = math.atan2(dy, dx)
            q = self.current_pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            error_ang = angle_to_target - yaw
            while error_ang > math.pi: error_ang -= 2 * math.pi
            while error_ang < -math.pi: error_ang += 2 * math.pi

            msg.linear.x = self.speed
            msg.angular.z = 1.0 * error_ang

        # 2. Control de Altitud (0.8m)
        if z < 0.75: msg.linear.z = 0.4
        elif z > 0.85: msg.linear.z = -0.4
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DronWander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
