#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import socket
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
        self.gz_pub = self.create_publisher(JointTrajectory, '/rover/arm_controller/joint_trajectory', 10)

        # === STATE ===
        self.target = None
        self.pose = None
        self.scan = None

        #===FLAGS PARA EL ROBOT STUDIO===
        self.rs_conectado = False
        self.objeto_disponible=False
        self.abb_activo=False

        #==CONFIGURACION DE SOCKET===
        self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip="172.26.112.1"
        self.puerto=5501
        self._conectar_socket()

        self.timer = self.create_timer(0.1, self.control_loop)
        #Leer el socket cada 10ms
        self.timer_socket = self.create_timer(0.01, self.leer_socket)

        self.get_logger().info("Nodo híbrido rover iniciado")

    def _conectar_socket(self):
        try:
            self.s.connect((self.ip, self.puerto))
            self.s.settimeout(2.0) #2seg para intentar conectar con el robotstudio
            saludo = self.s.recv(1024).decode()
            self.get_logger().info(f"RobotStudio dice: {saludo}")
            self.s.setblocking(False)
            self.rs_conectado=True
            self.get_logger().info("Socket conectado. Esperando llegada al objeto...")
        except Exception as e:
            self.get_logger().error(f"Error de conexión: {e}")
            self.rs_conectado=False
            

    # CALLBACKS
    def target_callback(self, msg):
        self.target = msg
        self.abb_activo=False
        self.objeto_disponible=False

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def lidar_callback(self, msg):
        self.scan = msg

    def leer_socket(self):
        if not self.abb_activo or not self.rs_conectado:
            return
        
        try:
            #Recibimos los del robotstudio datos
            data = self.s.recv(1024).decode()
            if data:
                if "HECHO" in data: #si en el mensaje viene la cadena "HECHO"
                    self.get_logger().info("Detectado fin de trayectoria")
                    self.abb_activo=False
                    self.objeto_disponible=False
                    data_limpia = data.replace("HECHO", "") #limpiamos el HECHO para no perder la posición
                    if data_limpia:
                        self.publicar_articulaciones(data_limpia)
                else:
                    self.publicar_articulaciones(data) #publicar
        except (BlockingIOError, socket.error):
            pass
        except Exception as e:
            self.get_logger().error(f"Error en lectura: {e}")

    def publicar_articulaciones(self, data):
        #Instancia de mensaje de trayectoria
        traj_msg = JointTrajectory()
        joint_names= ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6']
        traj_msg.joint_names = joint_names

        #Creación del punto de trayectoria
        point = JointTrajectoryPoint()

        try:
            angulos_grados = [float(x) for x in data.split(',')] #Paso de string a float
            radianes = [x * (3.14159 / 180.0) for x in angulos_grados] #ABB usa grados y ROS radianes --> pasar a radianes

            point.positions = radianes  #se define el punto objetivo
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 300000000 #0.3 seg para alcanzar la posicion

            #Esto es para que la ejecución sea instantánea
            traj_msg.header.stamp.sec = 0  
            traj_msg.header.stamp.nanosec = 0
            #Se añade el punto a la lista de puntos de la trayectoria
            traj_msg.points = [point]

            #envío del mensaje al controlador del brazo
            self.gz_pub.publish(traj_msg)
        
        except ValueError as e:
            self.get_logger().warn(f"Datos mal formateados: {data} → {e}")

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
            return 0.0, 0.0, float('inf')

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

        return twist.linear.x, twist.angular.z, dist

    # FUSION CONTROL
    def control_loop(self):
        if self.target is None or self.pose is None:
            return

        v_goal, w_goal, dist = self.compute_goal()
        v_obs, w_obs = self.compute_avoidance()

        if dist < 0.6 and not self.objeto_disponible and self.rs_conectado:
            self.get_logger().info("¡Objetivo alcanzado! Enviando OBJETO_DISPONIBLE al brazo")
            try:
                self.s.setblocking(True)
                self.s.send("OBJETO_DISPONIBLE".encode())
                self.s.setblocking(False)
                self.objeto_disponible = True
                self.abb_activo = True
            except Exception as e:
                self.get_logger().error(f"Error enviando OBJETO_DISPONIBLE: {e}")
                self.rs_conectado=False

        if self.abb_activo:
            self.cmd_pub.publish(Twist())
            return

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