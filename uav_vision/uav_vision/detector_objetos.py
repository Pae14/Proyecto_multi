import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import math
import time
from ament_index_python.packages import get_package_share_directory

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class DetectorDobleValidacion(Node):
    def __init__(self):
        super().__init__('detector_objetos')

        self.subscription = self.create_subscription(
            Image, '/uav/camera/image_raw', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/uav/odom', self.odom_callback, 10)
        self.rover_odom_sub = self.create_subscription(
            Odometry, '/rover/odom', self.rover_odom_callback, 10)

        self.target_pub = self.create_publisher(Point, '/uav/target_position', 10)
        self.verify_pub = self.create_publisher(Point, '/uav/verify_point', 10)
        self.drone_goto_pub = self.create_publisher(Point, '/uav/go_to', 10)

        self.bridge      = CvBridge()
        self.drone_pose  = None
        self.rover_pose  = None
        self.last_good_z = 0.8

        try:
            package_share_directory = get_package_share_directory('uav_vision')
            path_best = os.path.join(package_share_directory, 'weights', 'best.pt')
        except Exception:
            package_base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            path_best = os.path.join(package_base_path, 'weights', 'best.pt')

        self.model = YOLO(path_best) if os.path.exists(path_best) else None
        if self.model:
            self.model.model.names[0] = 'backpack'
            self.model.model.names[1] = 'toolbox'
            self.model.model.names[2] = 'dump'

        self.window_name = "Vision_IA_Dron"
        self.flow_window = "Estado_IA"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.flow_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.flow_window, 450, 750)
        
        self.processing   = False
        
        # Memoria de objetos
        self.objetos_detectados = [] 
        self.current_state = "IDLE"
        self.ultimo_evento = "SISTEMA INICIADO"
        self.tiempo_evento = 0

    def dibujar_flujo(self):
        canvas = np.zeros((750, 450, 3), dtype=np.uint8)
        canvas[:] = (25, 25, 25)
        
        # Header
        cv2.putText(canvas, "DASHBOARD CONTROL IA", (70, 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)
        cv2.line(canvas, (30, 55), (420, 55), (100, 100, 100), 1)

        estados = [
            ("BUSCANDO",    (50, 80, 350, 50),  (255, 150, 0)),
            ("DETECTADO",   (50, 150, 350, 50), (0, 255, 255)),
            ("REFINANDO",   (50, 220, 350, 50), (0, 165, 255)),
            ("ENVIANDO",    (50, 290, 350, 50), (0, 255, 0))
        ]

        state_map = {"BUSCANDO": "BUSCANDO", "YOLO_DETECT": "DETECTADO", "ESTABILIZANDO": "REFINANDO", "ENVIO_ROVER": "ENVIANDO"}
        active_label = state_map.get(self.current_state, "IDLE")

        for nombre, (x, y, w, h), color in estados:
            is_active = (nombre == active_label)
            bg_color = color if is_active else (45, 45, 45)
            cv2.rectangle(canvas, (x, y), (x+w, y+h), bg_color, -1)
            cv2.rectangle(canvas, (x, y), (x+w, y+h), (80, 80, 80), 1)
            cv2.putText(canvas, nombre, (x+120, y+32), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0) if is_active else (180,180,180), 2)

        # Panel de Notificaciones
        cv2.rectangle(canvas, (30, 370), (420, 430), (20, 40, 20), -1)
        cv2.rectangle(canvas, (30, 370), (420, 430), (0, 255, 0), 1)
        cv2.putText(canvas, f"[NOTIFICACION]: {self.ultimo_evento}", (45, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # Panel de Objetos en Memoria
        cv2.rectangle(canvas, (30, 460), (420, 710), (35, 35, 35), -1)
        cv2.putText(canvas, "OBJETOS CONFIRMADOS", (45, 490), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
        
        for i, obj in enumerate([o for o in self.objetos_detectados if o['confirmado']][:8]):
            y_pos = 530 + i*25
            cv2.circle(canvas, (55, y_pos-5), 5, (0, 255, 0), -1)
            txt = f"{obj['label'].upper()} -> X:{obj['x']:.2f} Y:{obj['y']:.2f}"
            cv2.putText(canvas, txt, (75, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

        cv2.imshow(self.flow_window, canvas)

    def odom_callback(self, msg):
        self.drone_pose = msg.pose.pose
        if msg.pose.pose.position.z > 0.1: self.last_good_z = msg.pose.pose.position.z

    def rover_odom_callback(self, msg):
        self.rover_pose = msg.pose.pose
        if self.rover_pose and self.objetos_detectados:
            rx, ry = self.rover_pose.position.x, self.rover_pose.position.y
            
            # Verificar si el rover ha llegado a algún objeto confirmado
            for obj in self.objetos_detectados:
                dist = math.sqrt((rx-obj['x'])**2 + (ry-obj['y'])**2)
                if dist < 1.2:
                    self.ultimo_evento = f"ROVER LLEGO A {obj['label'].upper()}"
                    self.get_logger().info(f"✅ {self.ultimo_evento}")
                    break
            
            # Limpiar lista
            self.objetos_detectados = [o for o in self.objetos_detectados if math.sqrt((rx-o['x'])**2 + (ry-o['y'])**2) > 1.2]

    def listener_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.current_state = "BUSCANDO"
            if self.processing or self.model is None or self.drone_pose is None:
                cv2.imshow(self.window_name, cv_image)
                self.dibujar_flujo()
                cv2.waitKey(1)
                return

            self.processing = True
            h, w = cv_image.shape[:2]
            results = self.model.predict(cv_image, conf=0.20, verbose=False)
            cv_image = results[0].plot()

            if results[0].boxes:
                self.current_state = "YOLO_DETECT"
                for box in results[0].boxes:
                    label = self.model.model.names.get(int(box.cls[0]), "obj")
                    if label in ['dump', 'backpack', 'toolbox']:
                        mx, my, dist_rel = self.proyectar_a_mundo(box.xywh[0][0], box.xywh[0][1] + (box.xywh[0][3]*0.4), w, h)
                        
                        if mx is not None and dist_rel < 10.0:
                            encontrado = False
                            for obj in self.objetos_detectados:
                                if math.sqrt((mx-obj['x'])**2 + (my-obj['y'])**2) < 3.0:
                                    encontrado = True
                                    self.current_state = "ESTABILIZANDO"
                                    
                                    peso = 0.5 if dist_rel < 3.0 else 0.1
                                    obj['x'] = (obj['x'] * (1-peso)) + (mx * peso)
                                    obj['y'] = (obj['y'] * (1-peso)) + (my * peso)
                                    obj['hits'] += 1
                                    
                                    if obj['hits'] >= 4 and not obj['confirmado']:
                                        obj['confirmado'] = True
                                        self.current_state = "ENVIO_ROVER"
                                        self.target_pub.publish(Point(x=float(obj['x']), y=float(obj['y']), z=0.0))
                                        self.ultimo_evento = f"ENVIADO: {obj['label'].upper()}"
                                    break
                            
                            if not encontrado:
                                self.objetos_detectados.append({'x': mx, 'y': my, 'hits': 1, 'confirmado': False, 'label': label})
                                self.drone_goto_pub.publish(Point(x=float(mx), y=float(my), z=1.5))
                                break

            cv2.imshow(self.window_name, cv_image)
            self.dibujar_flujo()
            cv2.waitKey(1)
        except Exception as e: self.get_logger().error(f'Error: {e}')
        finally: self.processing = False

    def _quat_to_rot(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])

    def proyectar_a_mundo(self, u, v, w, h):
        if not self.drone_pose: return None, None, 0
        altitude = self.drone_pose.position.z if self.drone_pose.position.z > 0.2 else self.last_good_z
        fov_h = np.deg2rad(90) 
        fx = fy = (w / 2.0) / math.tan(fov_h / 2.0)
        cx, cy = w / 2.0, h / 2.0
        r_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        tilt = np.deg2rad(12)
        ct, st = math.cos(tilt), math.sin(tilt)
        R_cam_body = np.array([[0, -st, ct], [-1, 0, 0], [0, -ct, -st]])
        R_bw = self._quat_to_rot(self.drone_pose.orientation)
        r_world = R_bw @ R_cam_body @ r_cam
        if r_world[2] >= -0.05: return None, None, 0
        cam_pos = np.array([self.drone_pose.position.x, self.drone_pose.position.y, altitude]) + R_bw @ np.array([0.15, 0.0, 0.0])
        t = -cam_pos[2] / r_world[2]
        if t < 0 or t > 15: return None, None, 0
        hit = cam_pos + t * r_world
        dist = math.sqrt((hit[0]-self.drone_pose.position.x)**2 + (hit[1]-self.drone_pose.position.y)**2)
        return float(hit[0]), float(hit[1]), dist

def main(args=None):
    rclpy.init(args=args)
    node = DetectorDobleValidacion()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
