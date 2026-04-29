import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import std_msgs.msg

class PuenteComunicacion(Node):
    def __init__(self):
        super().__init__('puente_abb')
        #Publica el estado de las articulaciones para ver como se mueven en rviz
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip="172.26.112.1"
        self.puerto=5501

        try:
            self.s.connect((self.ip, self.puerto))
            self.get_logger().info(f"Conectado a RobotStudio en {self.ip}")
        except Exception as e:
            self.get_logger().error(f"Error de conexión: {e}")


    def comunicacion_abb(self):
        try:
            #Informar a RobotStudio de que hay un objeto disponbible
            mensaje= "OBJETO_DISPONIBLE"
            print(f"Enviando comando: {mensaje}")
            self.s.send(mensaje.encode())

            while True:
                data = self.s.recv(1024).decode()
                if data == "HECHO":
                    break
                
                #RobotStudio envía el estado de las articulaciones
                self.publicar_articulaciones(data)

        except Exception as e:
            self.get_logger().error(f"Error durante la comunicación: {e}")
        finally:
            self.s.close()
            self.get_logger().info("Conexión con RobotStudio cerrada correctamente")

    
    def publicar_articulaciones(self, data):
        msg=JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        # Nombres de las articulaciones de tu XACRO
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        try:
            angulos_grados = [float(x) for x in data.split(',')] #Paso de string a float
            msg.position = [x * (3.14159 / 180.0) for x in angulos_grados] #ABB usa grados y ROS radianes --> pasar a radianes
            self.joint_pub.publish(msg)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node=PuenteComunicacion()
    node.comunicacion_abb()
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()

