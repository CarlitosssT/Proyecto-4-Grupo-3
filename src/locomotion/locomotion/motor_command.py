#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
Device.pin_factory = LGPIOFactory(chip=0)


# =========================
# Pines GPIO (BCM) — 3 puentes H (2 motores por puente)
# Cada puente agrupa sus 4 pines en filas físicas contiguas del header
# de 40 pines, para que el cableado hacia cada placa quede junto:
#   Puente H 1 (front): GPIO 5, 6, 13, 19  -> filas físicas 29,31,33,35
#   Puente H 2 (mid):   GPIO 17, 18, 27, 22 -> filas físicas 11,12,13,15
#   Puente H 3 (rear):  GPIO 12, 16, 20, 21 -> filas físicas 32,36,38,40
# Se dejan libres I2C (GPIO2,3), SPI0 (GPIO7,8,9,10,11) y UART (GPIO14,15)
# para un IMU.
# ¡AJUSTAR según el cableado real de cada puente H antes de energizar!
# =========================
MOTOR_PINS = {
    # lado izquierdo
    "front_left": (6, 5),      # puente H 1
    "mid_left":   (22, 17),    # puente H 2
    "rear_left":  (16, 12),    # puente H 3
    # lado derecho
    "front_right": (13, 19),   # puente H 1
    "mid_right":   (18, 23),   # puente H 2
    "rear_right":  (21, 20),   # puente H 3
}

LEFT_MOTORS = ("front_left", "mid_left", "rear_left")
RIGHT_MOTORS = ("front_right", "mid_right", "rear_right")


class MotorCommand(Node):
    def __init__(self):
        super().__init__("motor_command")

        self.cmd_subscription_ = self.create_subscription(
            Twist,
            "/motor_cmd",
            self.cmd_callback,
            10
        )

        self.latest_cmd_ = Twist()
        self.timer_ = self.create_timer(0.02, self.main_loop)  # 50 Hz

        self.motors_ = {
            name: Motor(forward=fwd, backward=bwd, pwm=True)
            for name, (fwd, bwd) in MOTOR_PINS.items()
        }

        self.get_logger().info("Nodo motor_command iniciado (6 motores / 3 puentes H).")
        self.get_logger().info("Suscrito a /motor_cmd")
        self.get_logger().info(f"Lado izquierdo: {LEFT_MOTORS}")
        self.get_logger().info(f"Lado derecho:   {RIGHT_MOTORS}")

    def cmd_callback(self, msg: Twist):
        self.latest_cmd_ = msg

    def main_loop(self):
        speed = self.latest_cmd_.linear.x
        turn = self.latest_cmd_.angular.z

        turn_gain = 0.7

        left_cmd = speed - turn_gain * turn
        right_cmd = speed + turn_gain * turn

        # ── Normalización: escala los dos lados juntos ──────────────
        # Si alguno supera 1.0, divide los DOS por ese máximo.
        # Así se preserva siempre la diferencia de velocidad (el giro).
        max_val = max(abs(left_cmd), abs(right_cmd), 1.0)
        left_cmd /= max_val
        right_cmd /= max_val

        # Todos los motores de un mismo lado reciben el mismo comando
        # (tracción tipo skid-steer con 3 motores por lado).
        for name in LEFT_MOTORS:
            self.motors_[name].value = left_cmd
        for name in RIGHT_MOTORS:
            self.motors_[name].value = right_cmd

    def destroy_node(self):
        for motor in self.motors_.values():
            motor.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommand()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for motor in node.motors_.values():
            motor.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
