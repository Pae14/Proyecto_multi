#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


class RoverHybridController(Node):

    def __init__(self):
        super().__init__('rover_hybrid_controller')

        # === SUBS ===
        self.create_subscription(Point, '/uav/target_position', self.target_callback, 10)
        self.create_subscription(Odometry, '/rover/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/rover/scan', self.lidar_callback, 10)

        # === PUB ===
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)

        # === STATE ===
        self.target = None
        self.pose = None
        self.scan = None

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Nodo híbrido rover iniciado")

    # CALLBACKS
    def target_callback(self, msg):
        self.target = msg

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def lidar_callback(self, msg):
        self.scan = msg

    # evitar obstaculos
    def compute_avoidance(self):
        if self.scan is None:
            return 0.0, 0.0

        ranges = list(self.scan.ranges)

        front = [r for r in ranges[0:30] + ranges[330:359] if 0.05 < r < 10.0]
        left  = [r for r in ranges[45:135] if 0.05 < r < 10.0]
        right = [r for r in ranges[225:315] if 0.05 < r < 10.0]

        d_front = min(front) if front else 10.0
        d_left  = min(left) if left else 10.0
        d_right = min(right) if right else 10.0

        twist = Twist()

        # Obstáculo frontal fuerte → prioridad alta
        if d_front < 1.0:
            twist.linear.x = -0.2
            twist.angular.z = 1.5 if d_left > d_right else -1.5
        elif d_front < 1.5:
            twist.linear.x = 0.0
            twist.angular.z = 0.8 if d_left > d_right else -0.8
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        return twist.linear.x, twist.angular.z

    # seguir el objetivo
    def compute_goal(self):
        if self.target is None or self.pose is None:
            return 0.0, 0.0

        dx = self.target.x - self.pose.position.x
        dy = self.target.y - self.pose.position.y

        dist = math.sqrt(dx**2 + dy**2)

        q = self.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                          1 - 2*(q.y*q.y + q.z*q.z))

        goal_angle = math.atan2(dy, dx)
        error = goal_angle - yaw

        # normalize
        while error > math.pi:
            error -= 2*math.pi
        while error < -math.pi:
            error += 2*math.pi

        twist = Twist()

        if dist > 0.6:
            twist.angular.z = max(min(1.2 * error, 1.2), -1.2)
            twist.linear.x = 1.5
        else:
            twist.linear.x = 0.2 * dist
            twist.angular.z = 1.0 * error

        return twist.linear.x, twist.angular.z

    # FUSION CONTROL
    def control_loop(self):
        if self.target is None or self.pose is None:
            return

        v_goal, w_goal = self.compute_goal()
        v_obs, w_obs = self.compute_avoidance()

        twist = Twist()

        # FUSIÓN 
        twist.linear.x = v_goal + v_obs * 1.2
        twist.angular.z = w_goal + w_obs * 2.0

        # saturación de seguridad
        twist.linear.x = max(min(twist.linear.x, 3.0), -0.5)
        twist.angular.z = max(min(twist.angular.z, 2.0), -2.0)

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = RoverHybridController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()