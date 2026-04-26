#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import time

class DronAutonomo(Node):
    def __init__(self):
        super().__init__('dron_autonomo_node')
        self.pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.create_subscription(Point, '/uav/target_position', self.target_callback, 10)
        
        self.target_detected = False
        self.start_time = time.time()
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('--- DRON AUTÓNOMO INICIADO: Vuelo estable ---')

    def target_callback(self, msg):
        if not self.target_detected:
            self.get_logger().info('¡BARRIL LOCALIZADO! Deteniendo búsqueda.')
            self.target_detected = True

    def control_loop(self):
        twist = Twist()
        elapsed = time.time() - self.start_time

        if self.target_detected:
            self.pub.publish(twist)
            return

        if elapsed < 10.0:
            # Fase 1: Despegue (Subida muy suave para evitar vibraciones)
            twist.linear.z = 0.8
        else:
            # Fase 2: Escaneo lento
            twist.linear.z = 0.1 
            twist.angular.z = 0.3 
        
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DronAutonomo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
