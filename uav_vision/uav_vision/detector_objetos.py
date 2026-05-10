import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import math
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class DetectorDobleValidacion(Node):
    def __init__(self):
        super().__init__('detector_objetos')
        
        self.subscription = self.create_subscription(Image, '/uav/camera/image_raw', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/uav/odom', self.odom_callback, 10)
        
        self.target_pub = self.create_publisher(Point, '/uav/target_position', 10)
        self.verify_pub = self.create_publisher(Point, '/uav/verify_point', 10)
        
        self.bridge = CvBridge()
        self.drone_pose = None
        
        # Cargar Modelos desde la nueva carpeta de pesos del proyecto
        # Rutas absolutas para garantizar la carga de los pesos
        path_yolo11 = '/home/paula/ros2_ws/src/proyecto_multi/uav_vision/weights/yolo11n-seg.pt'

        if os.path.exists(path_yolo11):
            self.get_logger().info(f'📦 Cargando pesos desde: {path_yolo11}')
            self.model_yolo11 = YOLO(path_yolo11)
        else:
            self.get_logger().error(f'❌ NO SE ENCONTRARON LOS PESOS EN: {path_yolo11}')
            self.model_yolo11 = None
        
        path_best = os.path.join(base_path, 'weights', 'best.pt')
        self.model_best = YOLO(path_best) if os.path.exists(path_best) else None

        self.get_logger().info('✅ SISTEMA DOBLE ACTIVADO: YOLO11 (Ojeador) + BEST.PT (Experto)')
        self.window_name = "Vision_IA_Dron"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.processing = False

    def odom_callback(self, msg):
        self.drone_pose = msg.pose.pose

    def listener_callback(self, data):
        if self.processing or not self.model_yolo11 or not self.drone_pose: return
            
        try:
            self.processing = True
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            h, w = frame.shape[:2]
            
            # 1. YOLO11 busca sospechosos (Confianza muy baja para ser sensible)
            res_11 = self.model_yolo11.predict(frame, conf=0.10, verbose=False)
            if res_11[0].boxes:
                box = res_11[0].boxes[0].xywh[0].cpu().numpy()
                mx, my, dist = self.proyectar_a_mundo(box[0], box[1], w, h)
                if mx is not None:
                    v_point = Point()
                    v_point.x, v_point.y = float(mx), float(my)
                    self.verify_pub.publish(v_point)

            # 2. BEST.PT confirma peligros
            res_best = self.model_best.predict(frame, conf=0.45, verbose=False)
            if res_best[0].boxes:
                for box in res_best[0].boxes:
                    label = self.model_best.names[int(box.cls[0])]
                    if label in ['barrel', 'backpack', 'toolbox', 'airplane']:
                        b = box.xywh[0].cpu().numpy()
                        mx, my, dist = self.proyectar_a_mundo(b[0], b[1], w, h)
                        if mx is not None:
                            target = Point()
                            target.x, target.y = float(mx), float(my)
                            self.target_pub.publish(target)
                            self.get_logger().info(f'🚨 PELIGRO [{label}] DETECTADO A {dist:.1f}m')
            
            cv2.imshow(self.window_name, res_best[0].plot())
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.processing = False

    def proyectar_a_mundo(self, u, v, w, h):
        # Focal para FOV ~70 deg
        f = w / (2 * np.tan(np.deg2rad(35))) 
        
        # Altura (dz): Usamos odometría, pero si es < 0.3 forzamos 0.8 para el cálculo
        dz = max(0.8, self.drone_pose.position.z)
        
        # Coordenadas relativas al centro de imagen
        xc = u - w/2
        yc = v - h/2  # Si yc > 0, el objeto está en la mitad inferior (en el suelo)
        
        # Ángulo de inclinación de la cámara (tilt): 15 grados hacia abajo
        tilt = np.deg2rad(15)
        
        # Ray-casting simplificado para cámara frontal inclinada
        # rz es la componente vertical del rayo en el mundo
        # rz = -f*sin(tilt) - yc*cos(tilt)
        rz = -f * math.sin(tilt) - yc * math.cos(tilt)
        
        if rz >= -0.1: return None, None, 0 # Apunta al cielo o horizonte
        
        # Factor de escala para llegar al suelo (z=0)
        k = -dz / rz
        
        # Coordenadas en el cuerpo del dron (X-adelante, Y-izq)
        rel_x = k * (f * math.cos(tilt) - yc * math.sin(tilt))
        rel_y = k * (-xc)
        
        dist = math.sqrt(rel_x**2 + rel_y**2)
        
        # Rotar al mapa global (Yaw del dron)
        q = self.drone_pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        mx = self.drone_pose.position.x + (rel_x * math.cos(yaw) - rel_y * math.sin(yaw))
        my = self.drone_pose.position.y + (rel_x * math.sin(yaw) + rel_y * math.cos(yaw))
        
        return mx, my, dist

def main(args=None):
    rclpy.init(args=args)
    node = DetectorDobleValidacion()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
