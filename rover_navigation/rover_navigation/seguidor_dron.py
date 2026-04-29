#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import math

class SeguidorDron(Node):
    def __init__(self):
        super().__init__('seguidor_dron_node')
        
        # Subscripciones
        self.create_subscription(Point, '/uav/target_position', self.target_callback, 10)
        self.create_subscription(Odometry, '/rover/odom', self.odom_callback, 10)
        
        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        
        self.target_global = None
        self.rover_pose = None
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('--- COORDINADOR ROVER LISTO: Siguiendo coordenadas del dron ---')

    def target_callback(self, msg):
        self.target_global = msg
        self.get_logger().info(f'Nuevo objetivo recibido: X={msg.x:.2f}, Y={msg.y:.2f}')

    def odom_callback(self, msg):
        self.rover_pose = msg.pose.pose

    def control_loop(self):
        if self.target_global is None or self.rover_pose is None:
            return

        # Calcular vector relativo del Rover al Objetivo
        dx = self.target_global.x - self.rover_pose.position.x
        dy = self.target_global.y - self.rover_pose.position.y
        
        distancia = math.sqrt(dx**2 + dy**2)
        
        # Obtener orientación actual del Rover (yaw)
        q = self.rover_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Ángulo hacia el objetivo
        angulo_objetivo = math.atan2(dy, dx)
        error_angulo = angulo_objetivo - yaw
        
        # Normalizar ángulo (-pi a pi)
        while error_angulo > math.pi: error_angulo -= 2 * math.pi
        while error_angulo < -math.pi: error_angulo += 2 * math.pi

        twist = Twist()

        if distancia > 0.5:
            # Si el ángulo es muy grande, primero girar sobre sí mismo
            if abs(error_angulo) > 0.5:
                twist.linear.x = 0.0
                twist.angular.z = 0.8 * error_angulo
            else:
                twist.linear.x = 0.6
                twist.angular.z = 1.0 * error_angulo
            
            self.get_logger().info(f'Navegando: dist={distancia:.2f}, error_ang={error_angulo:.2f}')
        else:
            self.get_logger().info('¡OBJETIVO ALCANZADO!')
            self.target_global = None
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SeguidorDron()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
