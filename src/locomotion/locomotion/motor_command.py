#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from locomotion.pca9685 import PCA9685


# =========================
# Canales del PCA9685 — 3 puentes H (2 motores por puente)
# Cada motor usa (IN_forward, IN_backward). Por puente: IN1,IN2 = motor
# izquierdo; IN3,IN4 = motor derecho.
#   Puente H 1 (front): IN1..IN4 = canales 0, 1, 2, 3
#   Puente H 2 (mid):   IN1..IN4 = canales 4, 5, 6, 7
#   Puente H 3 (rear):  IN1..IN4 = canales 8, 9, 10, 11
# Canales 12..15 libres.
# Si un motor gira al revés, intercambia sus dos canales aquí (o sus cables).
# ¡AJUSTAR según el cableado real de cada puente H antes de energizar!
# =========================
MOTOR_CHANNELS = {
    # lado izquierdo (IN1, IN2)
    "front_left": (0, 1),    # puente H 1
    "mid_left":   (4, 5),    # puente H 2
    "rear_left":  (8, 9),    # puente H 3
    # lado derecho (IN3, IN4)
    "front_right": (2, 3),   # puente H 1
    "mid_right":   (6, 7),   # puente H 2
    "rear_right":  (10, 11), # puente H 3
}

LEFT_MOTORS = ("front_left", "mid_left", "rear_left")
RIGHT_MOTORS = ("front_right", "mid_right", "rear_right")


class PCAMotor:
    """Motor DC en un puente H con PWM sobre IN_forward o IN_backward
    (mismo comportamiento que gpiozero.Motor con pwm=True)."""

    def __init__(self, pca, forward, backward):
        self.pca_ = pca
        self.forward_ = forward
        self.backward_ = backward
        self.value_ = None  # fuerza la primera escritura

    @property
    def value(self):
        return self.value_ or 0.0

    @value.setter
    def value(self, value):
        value = max(-1.0, min(1.0, value))
        if value == self.value_:
            return  # evita tráfico I2C innecesario
        self.pca_.set_duty(self.forward_, max(value, 0.0))
        self.pca_.set_duty(self.backward_, max(-value, 0.0))
        self.value_ = value

    def stop(self):
        self.value = 0.0


class MotorCommand(Node):
    def __init__(self):
        super().__init__("motor_command")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x40)
        self.declare_parameter("pwm_frequency", 1000)

        self.pca_ = PCA9685(
            bus=self.get_parameter("i2c_bus").value,
            address=self.get_parameter("i2c_address").value,
            frequency=self.get_parameter("pwm_frequency").value,
        )

        self.cmd_subscription_ = self.create_subscription(
            Twist,
            "/motor_cmd",
            self.cmd_callback,
            10
        )

        self.latest_cmd_ = Twist()
        self.timer_ = self.create_timer(0.02, self.main_loop)  # 50 Hz

        self.motors_ = {
            name: PCAMotor(self.pca_, fwd, bwd)
            for name, (fwd, bwd) in MOTOR_CHANNELS.items()
        }

        self.get_logger().info("Nodo motor_command iniciado (6 motores / 3 puentes H vía PCA9685).")
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
        #self.motors_["rear_right"].value = left_cmd


    def destroy_node(self):
        for motor in self.motors_.values():
            motor.stop()
        self.pca_.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommand()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
