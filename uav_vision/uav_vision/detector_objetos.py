import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import math
from ament_index_python.packages import get_package_share_directory


from ultralytics import YOLO


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
        
        # Obtener la ruta de los pesos usando el directorio compartido del paquete
        try:
            package_share_directory = get_package_share_directory('uav_vision')
            path_yolo11 = os.path.join(package_share_directory, 'weights', 'yolo11n-seg.pt')
            path_best = os.path.join(package_share_directory, 'weights', 'best.pt')
        except Exception:
            # Fallback por si no está instalado (uso directo en src)
            self.get_logger().warn('Paquete no encontrado en install, buscando en ruta local...')
            package_base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            path_yolo11 = os.path.join(package_base_path, 'weights', 'yolo11n-seg.pt')
            path_best = os.path.join(package_base_path, 'weights', 'best.pt')

        self.get_logger().info(f'📦 Cargando pesos desde: {path_yolo11}')
        self.model_yolo11 = YOLO(path_yolo11) if os.path.exists(path_yolo11) else None
        self.model_best = YOLO(path_best) if os.path.exists(path_best) else None

        if not self.model_yolo11:
            self.get_logger().error('❌ FALLO CRÍTICO: No se pudo cargar yolo11n-seg.pt')

        self.get_logger().info('✅ SISTEMA DE VISIÓN INICIADO')
        self.window_name = "Vision_IA_Dron"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.processing = False

    def odom_callback(self, msg):
        self.drone_pose = msg.pose.pose

    def target_callback(self, msg):
        if self.pose is not None and self.target is not None:
            dx = self.target.x - self.pose.position.x
            dy = self.target.y - self.pose.position.y
            dist_actual = math.sqrt(dx**2 + dy**2)
            if dist_actual < 2.5:  # congelar target dentro de 2.5m
                return
        self.target = msg

    def listener_callback(self, data):
        try:
            # Convertir imagen
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            # Si ya estamos procesando un frame o no hay modelo, solo mostramos la imagen actual
            if self.processing or not self.model_yolo11:
                cv2.imshow(self.window_name, cv_image)
                cv2.waitKey(1)
                return

            self.processing = True
            
            # Solo procesamos IA si tenemos la posición del dron
            if self.drone_pose:
                h, w = cv_image.shape[:2]
                
                # 1. YOLO11 busca sospechosos
                res_11 = self.model_yolo11.predict(cv_image, conf=0.10, verbose=False)
                if res_11[0].boxes:
                    box = res_11[0].boxes[0].xywh[0].cpu().numpy()
                    mx, my, dist = self.proyectar_a_mundo(box[0], box[1], w, h)
                    if mx is not None:
                        v_point = Point()
                        v_point.x, v_point.y = float(mx), float(my)
                        self.verify_pub.publish(v_point)

                # 2. BEST.PT confirma peligros
                if self.model_best:
                    res_best = self.model_best.predict(cv_image, conf=0.45, verbose=False)
                    cv_image = res_best[0].plot()
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
                                    self.get_logger().info(f'🚨 PELIGRO [{label}] DETECTADO')
                else:
                    cv_image = res_11[0].plot()

            # MOSTRAR SIEMPRE LA VENTANA
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error en el nodo de visión: {e}')
        finally:
            self.processing = False

    def proyectar_a_mundo(self, u, v, w, h):
        if not self.drone_pose: return None, None, 0
        f = w / (2 * np.tan(np.deg2rad(35))) 
        dz = max(0.8, self.drone_pose.position.z)
        xc, yc = u - w/2, v - h/2
        tilt = np.deg2rad(15)
        rz = -f * math.sin(tilt) - yc * math.cos(tilt)
        if rz >= -0.1: return None, None, 0 
        k = -dz / rz
        rel_x = k * (f * math.cos(tilt) - yc * math.sin(tilt)) + 0.8
        rel_y = k * (-xc)
        dist = math.sqrt(rel_x**2 + rel_y**2)
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
