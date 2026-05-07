import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import socket
import std_msgs.msg

class PuenteComunicacion(Node):
    def __init__(self):
        super().__init__('puente_abb')
        #Publica el estado de las articulaciones para ver como se mueven en rviz
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        #Publica el estado de las articulaciones para gazebo
        self.gz_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip="172.26.112.1"
        self.puerto=5501

        try:
            self.s.connect((self.ip, self.puerto))
            self.s.settimeout(10.0)
            saludo = self.s.recv(1024).decode()
            self.get_logger().info(f"RobotStudio dice: {saludo}")
            self.s.send("OBJETO_DISPONIBLE".encode())
            self.s.setblocking(False)
            self.get_logger().info(f"Conectado a RobotStudio en {self.ip}")
        except Exception as e:
            self.get_logger().error(f"Error de conexión: {e}")

        self.timer = self.create_timer(0.01, self.leer_socket)


    def leer_socket(self):
        try:
            # Intentamos recibir datos
            data = self.s.recv(1024).decode()
            if data:
                if "HECHO" in data:
                    self.get_logger().info("Detectado fin de trayectoria (HECHO)")
                    data_limpia = data.replace("HECHO", "")
                    if data_limpia:
                        self.publicar_articulaciones(data_limpia)
                else:
                    self.publicar_articulaciones(data)
        except (BlockingIOError, socket.error):
            pass
        except Exception as e:
            self.get_logger().error(f"Error en lectura: {e}")

    
    def publicar_articulaciones(self, data):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        traj_msg = JointTrajectory()
        traj_msg.joint_names = msg.name
        point = JointTrajectoryPoint()

        try:
            angulos_grados = [float(x) for x in data.split(',')] #Paso de string a float
            radianes = [x * (3.14159 / 180.0) for x in angulos_grados] #ABB usa grados y ROS radianes --> pasar a radianes
            
            #Publicar en Rviz
            msg.position = radianes
            self.joint_pub.publish(msg)

            #Publicar en Gazebo
            point.positions=radianes
            point.time_from_start.sec = 2
            traj_msg.points.append(point)
            self.gz_pub.publish(traj_msg)

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

