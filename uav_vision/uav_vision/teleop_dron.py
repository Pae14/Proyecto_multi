#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control del Dron
---------------------------
Moverse:          Subir/Bajar:
   u    i    o       t
   j    k    l       b
   m    ,    .

i : Adelante
, : Atrás
j : Girar Izquierda
l : Girar Derecha
t : SUBIR
b : BAJAR
k : PARAR

q/z : Aumentar/Disminuir velocidad lineal (0.5)
w/x : Aumentar/Disminuir velocidad angular (1.0)

CTRL-C para salir
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('teleop_dron_custom')
    pub = node.create_publisher(Twist, '/uav/cmd_vel', 10)

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0

    try:
        print(msg)
        while True:
            key = get_key(settings)
            if key == 'i': x = 1.0
            elif key == ',': x = -1.0
            elif key == 'j': th = 1.0
            elif key == 'l': th = -1.0
            elif key == 't': z = 1.0
            elif key == 'b': z = -1.0
            elif key == 'k':
                x = y = z = th = 0.0
            elif key == 'q': speed *= 1.1
            elif key == 'z': speed *= 0.9
            elif key == 'w': turn *= 1.1
            elif key == 'x': turn *= 0.9
            elif key == '\x03': break
            else:
                x = y = z = th = 0.0

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
