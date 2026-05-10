import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import socket
import std_msgs.msg

class PuenteComunicacion(Node):
    def __init__(self):
        super().__init__('puente_abb')

        #Publicador para las trayectorias para gazebo
        self.gz_pub = self.create_publisher(JointTrajectory, '/rover/arm_controller/joint_trajectory', 10)
        
        #Configuracion del socket
        self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip="172.26.112.1"
        self.puerto=5501

        try:
            #Conexión del socket
            self.s.connect((self.ip, self.puerto))
            self.s.settimeout(10.0)
            saludo = self.s.recv(1024).decode()
            self.get_logger().info(f"RobotStudio dice: {saludo}")
            self.s.send("OBJETO_DISPONIBLE".encode())
            #Socket no bloqueante para no congelar el nodo de ros mientras espera datos
            self.s.setblocking(False)
            self.get_logger().info(f"Conectado a RobotStudio en {self.ip}")
        except Exception as e:
            self.get_logger().error(f"Error de conexión: {e}")

        #Leer el socket cada 10ms
        self.timer = self.create_timer(0.01, self.leer_socket)


    def leer_socket(self):
        try:
            #Recibimos los del robotstudio datos
            data = self.s.recv(1024).decode()
            if data:
                if "HECHO" in data: #si en el mensaje viene la cadena "HECHO"
                    self.get_logger().info("Detectado fin de trayectoria (HECHO)")
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
            self.get_logger().info(f"Publicando: {[f'{r:.3f}' for r in radianes]}")

        except ValueError as e:
            self.get_logger().warn(f"Datos mal formateados: {data} → {e}")

def main(args=None):
    rclpy.init(args=args)
    node=PuenteComunicacion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


