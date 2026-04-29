import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import time

class DetectorObjetos(Node):
    def __init__(self):
        super().__init__('detector_objetos')
        
        self.subscription = self.create_subscription(Image, '/uav/camera/image_raw', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/uav/odom', self.odom_callback, 10)
        self.target_pub = self.create_publisher(Point, '/uav/target_position', 10)
        
        self.bridge = CvBridge()
        self.drone_pose = None
        self.detection_counter = 0
        
        model_path = '/home/paula/ros2_ws/yolo11n-seg.pt'
        if not os.path.exists(model_path):
            model_path = '/home/paula/ros2_ws/yolov8n-seg.pt'
            
        if YOLO is not None and os.path.exists(model_path):
            self.model = YOLO(model_path)
            self.get_logger().info('Visión Inteligente Lista.')
        else:
            self.model = None

        self.window_name = "Vision_IA_Dron"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 480)
        self.processing = False
        self.last_goal_time = 0

    def odom_callback(self, msg):
        if self.drone_pose is None:
            self.get_logger().info('¡Odometría del Dron RECIBIDA!')
        self.drone_pose = msg.pose.pose

    def listener_callback(self, data):
        if self.processing:
            return
            
        try:
            self.processing = True
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            if self.model is not None:
                # Umbral a 0.20 para máxima detección
                results = self.model.predict(frame, conf=0.20, verbose=False)
                annotated_frame = results[0].plot()
                
                danger_found = False
                for box in results[0].boxes:
                    label = self.model.names[int(box.cls[0])]
                    if label in ['fire hydrant', 'bottle', 'stop sign', 'bus', 'truck', 'cup', 'person']:
                        danger_found = True
                        break
                
                if danger_found:
                    self.detection_counter += 1
                    if self.detection_counter > 2:
                        if time.time() - self.last_goal_time > 4:
                            self.enviar_a_rover()
                            self.last_goal_time = time.time()
                else:
                    self.detection_counter = 0
                
                display_frame = cv2.resize(annotated_frame, (640, 480))
                cv2.imshow(self.window_name, display_frame)
            
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.processing = False

    def enviar_a_rover(self):
        msg = Point()
        if self.drone_pose is not None:
            msg.x = self.drone_pose.position.x
            msg.y = self.drone_pose.position.y
            self.get_logger().warn(f'>>> PELIGRO: Enviando posición REAL: X={msg.x:.2f}, Y={msg.y:.2f}')
        else:
            msg.x = 5.0
            msg.y = 0.0
            self.get_logger().error('>>> SIN ODOMETRÍA: Usando posición estimada...')
            
        msg.z = 0.0
        self.target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorObjetos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
