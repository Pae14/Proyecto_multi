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

class DetectorObjetos(Node):
    def __init__(self):
        super().__init__('detector_objetos')
        
        # Tópico configurado en el bridge de Gazebo
        self.subscription = self.create_subscription(
            Image,
            '/uav/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        
        # Cargar modelo YOLO (prioridad a YOLOv11)
        model_path = '/home/paula/ros2_ws/yolo11n-seg.pt'
        if not os.path.exists(model_path):
            model_path = '/home/paula/ros2_ws/yolov8n-seg.pt'
            
        if YOLO is not None and os.path.exists(model_path):
            self.model = YOLO(model_path)
            self.get_logger().info(f'Modelo cargado: {model_path}')
        else:
            self.model = None
            if YOLO is None:
                self.get_logger().error('Librería "ultralytics" no instalada. Instálala con: pip install ultralytics')
            else:
                self.get_logger().error(f'No se encontró el modelo en {model_path}')

        self.get_logger().info('Nodo de visión iniciado. Esperando imágenes...')

    def listener_callback(self, data):
        try:
            # Convertir imagen ROS a OpenCV
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # 1. Filtro inicial para reducir ruido
            frame_filtrado = cv2.GaussianBlur(frame, (5, 5), 0)

            if self.model is not None:
                # 2. Inferencia con YOLO (Segmentación)
                results = self.model(frame_filtrado, verbose=False)
                
                # 3. Dibujar resultados (máscaras y cajas)
                annotated_frame = results[0].plot()
                
                # Mostrar resultado segmentado
                cv2.imshow("Segmentación UAV (YOLO)", annotated_frame)
            else:
                # Si no hay modelo, mostrar solo el filtro
                cv2.imshow("Vista Dron - Solo Filtro (Sin YOLO)", frame_filtrado)

            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error al procesar imagen: {e}')

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
