#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class Wander(Node):

    def __init__(self):
        super().__init__('wander')

        self.declare_parameter('robot', 'robot1')
        robot = self.get_parameter('robot').get_parameter_value().string_value

        self.subscription = self.create_subscription(LaserScan, f'/{robot}/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(Twist, f'/{robot}/cmd_vel', 10)

        # --- NUEVAS VARIABLES DE CONTROL ---
        self.girando = False        # Indica si estamos en "modo giro forzado"
        self.tiempo_inicio_giro = 0  # Cuándo empezamos a girar
        self.duracion_giro = 2    # Segundos que queremos que gire (ajusta según necesites)
        self.direccion_giro = 1.0   # 1.0 para izquierda, -1.0 para derecha

        self.get_logger().info(f'Iniciando robot: {robot} con temporizador de giro.')

    def lidar_callback(self, msg):
        ahora = self.get_clock().now().nanoseconds / 1e9 # Tiempo actual en segundos
        msg_vel = Twist()

        # 1. ¿ESTAMOS EN MODO GIRO FORZADO?
        if self.girando:
            if ahora - self.tiempo_inicio_giro < self.duracion_giro:
                # Seguimos girando sin mirar el sensor
                msg_vel.linear.x = 0.0
                msg_vel.angular.z = 0.8 * self.direccion_giro
                self.publisher.publish(msg_vel)
                return # Salimos del callback para no evaluar lo demás
            else:
                # Ya pasó el tiempo, volvemos al modo normal
                self.girando = False
                self.get_logger().info("Giro completado, reanudando marcha.")

        # 2. PROCESAMIENTO NORMAL DE SENSORES
        ranges = list(msg.ranges)
        # Ajustamos los índices (frente: 30 grados a cada lado)
        # Si tu lidar tiene 360 puntos:
        frente = [r for r in (ranges[0:30] + ranges[330:359]) if 0.05 < r < 10.0]
        izquierda = [r for r in ranges[45:135] if 0.05 < r < 10.0]
        derecha = [r for r in ranges[225:315] if 0.05 < r < 10.0]

        dist_frente = min(frente) if frente else 10.0
        dist_izq = min(izquierda) if izquierda else 10.0
        dist_der = min(derecha) if derecha else 10.0

        # 3. LÓGICA DE ACTIVACIÓN DEL GIRO
        if dist_frente < 1.2:
            # ¡Obstáculo detectado! Activamos el modo "Giro Forzado"
            self.girando = True
            self.tiempo_inicio_giro = ahora
            # Decidimos dirección basada en qué lado está más despejado
            self.direccion_giro = 1.0 if dist_izq > dist_der else -1.0
            self.get_logger().warn(f"Obstáculo a {dist_frente:.2f}m. Girando por {self.duracion_giro}s")
            
            # Frenazo inicial y primer giro
            msg_vel.linear.x = -0.1 # Un pelín de marcha atrás para no rozar
            msg_vel.angular.z = 0.8 * self.direccion_giro
        else:
            # Camino despejado
            msg_vel.linear.x = 0.3
            msg_vel.angular.z = 0.0

        self.publisher.publish(msg_vel)

def main(args=None):
    rclpy.init(args=args)
    nodo = Wander()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.publisher.publish(Twist()) # Parar robot
        nodo.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()