#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import math

class SeguidorDron(Node):
    def __init__(self):
        super().__init__('seguidor_dron_node')
        self.create_subscription(Point, '/uav/target_position', self.target_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.target = None
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('--- COORDINADOR ROVER LISTO: Aumentando potencia ---')

    def target_callback(self, msg):
        self.target = msg

    def control_loop(self):
        if self.target is None:
            return

        distancia = math.sqrt(self.target.x**2 + self.target.y**2)
        twist = Twist()

        if distancia > 1.0:
            error_angulo = math.atan2(self.target.y, self.target.x)
            twist.linear.x = 0.8  # POTENCIA AUMENTADA
            twist.angular.z = 1.2 * error_angulo
            self.get_logger().info(f'Moviendo Rover: dist={distancia:.2f}, ang={error_angulo:.2f}')
        else:
            self.get_logger().info('¡BARRIL ALCANZADO!')
            self.target = None

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
