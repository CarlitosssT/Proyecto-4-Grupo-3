#!/usr/bin/env python3
"""
Teleoperación por teclado — publica en /motor_cmd (mismo tópico que rc_control,
así que reutiliza la mezcla izquierda/derecha ya implementada en motor_command).

Controles:
  w / s   → aumentar / disminuir velocidad lineal
  a / d   → girar izquierda / derecha
  espacio → detener (velocidad y giro a 0)
  q       → salir
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_STEP = 0.5
ANGULAR_STEP = 0.3
LINEAR_MAX = 5.0
ANGULAR_MAX = 3.0

HELP = """
─────────────────────────────────────────
  Teleop teclado — locomotion
─────────────────────────────────────────
  w/s     : velocidad lineal +/-
  a/d     : giro izquierda/derecha
  espacio : detener
  q       : salir
─────────────────────────────────────────
"""


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__("keyboard_control_node")
        self.motor_pub_ = self.create_publisher(Twist, "/motor_cmd", 10)
        self.linear_ = 0.0
        self.angular_ = 0.0
        self.get_logger().info(HELP)

    def clamp(self, value, vmin, vmax):
        return max(vmin, min(vmax, value))

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear_
        msg.angular.z = self.angular_
        self.motor_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == 'w':
                node.linear_ = node.clamp(node.linear_ + LINEAR_STEP, -LINEAR_MAX, LINEAR_MAX)
            elif key == 's':
                node.linear_ = node.clamp(node.linear_ - LINEAR_STEP, -LINEAR_MAX, LINEAR_MAX)
            elif key == 'a':
                node.angular_ = node.clamp(node.angular_ + ANGULAR_STEP, -ANGULAR_MAX, ANGULAR_MAX)
            elif key == 'd':
                node.angular_ = node.clamp(node.angular_ - ANGULAR_STEP, -ANGULAR_MAX, ANGULAR_MAX)
            elif key == ' ':
                node.linear_ = 0.0
                node.angular_ = 0.0
            elif key in ('q', '\x03'):  # q o Ctrl+C
                break
            else:
                continue

            node.publish_cmd()
            node.get_logger().info(
                f"linear={node.linear_:+.2f}  angular={node.angular_:+.2f}"
            )
    finally:
        node.linear_ = 0.0
        node.angular_ = 0.0
        node.publish_cmd()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
