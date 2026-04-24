import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorObjetos(Node):
    def __init__(self):
        super().__init__('detector_objetos')
        # Tópico de cámara genérico. Rol 2 te confirmará el nombre exacto 
        # cuando configure los namespaces del dron.
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Nodo de visión iniciado. Esperando imágenes...')

    def listener_callback(self, data):
        try:
            # Convertir imagen ROS a formato OpenCV (BGR)
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # --- ROL 1: IMPLEMENTACIÓN DE LA RED NEURONAL ---
            # 1. Filtro inicial con kernels grandes para eliminar texturas finas
            # tal como se especifica en la misión del Rol 1.
            kernel_size = (15, 15)
            frame_filtrado = cv2.GaussianBlur(frame, kernel_size, 0)

            # 2. Aquí integrarás tu arquitectura CNN (TensorFlow/PyTorch)
            # Por ahora, simulamos una detección visual simple
            
            # --- FIN DE LA SECCIÓN DE IA ---

            # Visualización para depuración
            cv2.imshow("Vista Dron - Procesada (Rol 1)", frame_filtrado)
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
