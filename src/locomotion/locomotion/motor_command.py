#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Int32

from locomotion.pca9685 import PCA9685


# =========================
# Canales del PCA9685 — 3 puentes H TB6612FNG (2 motores por puente)
# PWMA y PWMB van fijos a VCC (3.3 V): el PWM se aplica sobre AIN1/AIN2 y
# BIN1/BIN2 (IN1=PWM, IN2=L -> avance/libre; IN1=L, IN2=PWM -> reversa/libre).
# Cada motor usa (IN_forward, IN_backward). Por puente: A = motor izquierdo,
# B = motor derecho.
#   Puente H 1 (front): AIN1, AIN2, BIN1, BIN2 = canales 0, 1, 2, 3
#   Puente H 2 (mid):   AIN1, AIN2, BIN1, BIN2 = canales 4, 5, 6, 7
#   Puente H 3 (rear):  AIN1, AIN2, BIN1, BIN2 = canales 8, 9, 10, 11
# STBY de los 3 puentes va fijo a 5 V (siempre habilitados).
# Canal 12: servo de inclinación de la cámara (parámetro cam_channel).
# Canal 13: servo de la pala de excavación (parámetro shovel_channel).
# Canales 14..15 libres.
# Si un motor gira al revés, intercambia sus dos canales aquí (o sus cables).
# ¡AJUSTAR según el cableado real de cada puente H antes de energizar!
# =========================
MOTOR_CHANNELS = {
    # lado izquierdo (AIN1, AIN2)
    "front_left": (0, 1),    # puente H 1
    "mid_left":   (4, 5),    # puente H 2
    "rear_left":  (8, 9),    # puente H 3
    # lado derecho (BIN1, BIN2)
    "front_right": (2, 3),   # puente H 1
    "mid_right":   (6, 7),   # puente H 2
    "rear_right":  (10, 11), # puente H 3
}

LEFT_MOTORS = ("front_left", "mid_left", "rear_left")
RIGHT_MOTORS = ("front_right", "mid_right", "rear_right")


class PCAMotor:
    """Motor DC en un puente H con PWM sobre IN_forward o IN_backward
    (el otro IN queda en bajo)."""

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


class PCAServo:
    """Servo en un canal del PCA9685. El chip tiene UNA frecuencia para los 16
    canales: con servos tiene que ir a ~50 Hz (periodo de 20 ms)."""

    def __init__(self, pca, channel, min_deg, max_deg, start_deg,
                 pulse_min_us=500, pulse_max_us=2400, deg_per_tick=4):
        self.pca_ = pca
        self.channel_ = channel
        self.min_ = min_deg
        self.max_ = max_deg
        self.pulse_min_ = pulse_min_us
        self.pulse_max_ = pulse_max_us
        self.deg_per_tick_ = deg_per_tick   # rampa de goto() por llamada a update()
        self.angle_ = self.clamp(start_deg)
        self.target_ = self.angle_
        self._write()

    def clamp(self, deg):
        return max(self.min_, min(self.max_, deg))

    def _write(self):
        # Mapeo 0..180° -> pulse_min..pulse_max (mismo rango que el servo de la ESP)
        us = self.pulse_min_ + (self.pulse_max_ - self.pulse_min_) * self.angle_ / 180.0
        self.pca_.set_pulse_us(self.channel_, us)

    @property
    def angle(self):
        return self.angle_

    @property
    def target(self):
        return self.target_

    def nudge(self, delta):
        """Movimiento del stick: pasos chicos, se escriben directo."""
        new = self.clamp(self.angle_ + delta)
        self.target_ = new                  # cancela un goto() en curso
        if new != self.angle_:
            self.angle_ = new
            self._write()

    def goto(self, deg):
        """Ir a un ángulo con rampa (la hace update())."""
        self.target_ = self.clamp(deg)

    def update(self):
        if self.angle_ == self.target_:
            return
        step = max(-self.deg_per_tick_, min(self.deg_per_tick_, self.target_ - self.angle_))
        self.angle_ += step
        self._write()

    def release(self):
        self.pca_.set_duty(self.channel_, 0.0)  # sin pulsos: el servo queda sin fuerza


class MotorCommand(Node):
    def __init__(self):
        super().__init__("motor_command")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x40)
        # 50 Hz porque el servo de la cámara comparte el PCA con los motores
        # (antes eran 1000 Hz; los TB6612 funcionan igual, quizá zumben un poco).
        self.declare_parameter("pwm_frequency", 50)
        # Seguridad: sin /armed=True (rc_control) o sin /motor_cmd reciente, frena
        self.declare_parameter("require_arm", True)
        self.declare_parameter("cmd_timeout", 0.5)    # s; <= 0 lo desactiva
        self.declare_parameter("cam_channel", 12)
        self.declare_parameter("cam_min_deg", 20)     # ¡ajustar al montaje real!
        self.declare_parameter("cam_max_deg", 160)
        self.declare_parameter("cam_start_deg", 90)
        self.declare_parameter("cam_pulse_min_us", 500)
        self.declare_parameter("cam_pulse_max_us", 2400)
        # Servo de la pala (antes en la ESP32): mismos límites, arranque y pulsos
        self.declare_parameter("shovel_channel", 13)
        self.declare_parameter("shovel_min_deg", 5)
        self.declare_parameter("shovel_max_deg", 130)
        self.declare_parameter("shovel_start_deg", 30)
        self.declare_parameter("shovel_pulse_min_us", 500)
        self.declare_parameter("shovel_pulse_max_us", 2400)

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
        self.last_cmd_time_ = None
        self.armed_ = False
        self.require_arm_ = bool(self.get_parameter("require_arm").value)
        self.cmd_timeout_ = float(self.get_parameter("cmd_timeout").value)
        self.stop_reason_ = None    # para loguear solo los cambios

        # Latched: recibe el estado actual aunque rc_control haya arrancado antes
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/armed", self.armed_callback, latched)

        if self.pca_.frequency > 100:
            self.get_logger().warn(
                f"pwm_frequency={self.pca_.frequency:.0f} Hz: un servo estándar "
                "necesita ~50 Hz, la cámara y la pala no se van a mover bien.")
        self.cam_ = PCAServo(
            self.pca_,
            channel=self.get_parameter("cam_channel").value,
            min_deg=self.get_parameter("cam_min_deg").value,
            max_deg=self.get_parameter("cam_max_deg").value,
            start_deg=self.get_parameter("cam_start_deg").value,
            pulse_min_us=self.get_parameter("cam_pulse_min_us").value,
            pulse_max_us=self.get_parameter("cam_pulse_max_us").value,
        )
        self.create_subscription(Int32, "/camera_tilt_delta",
                                 lambda m: self.cam_.nudge(m.data), 10)
        self.create_subscription(Int32, "/camera_tilt_angle",
                                 lambda m: self.cam_.goto(m.data), 10)

        # Pala: siempre con rampa (va cargada). 1°/ciclo a 50 Hz = 50 °/s,
        # parecido a los 15 ms por grado que usaba la ESP.
        self.shovel_ = PCAServo(
            self.pca_,
            channel=self.get_parameter("shovel_channel").value,
            min_deg=self.get_parameter("shovel_min_deg").value,
            max_deg=self.get_parameter("shovel_max_deg").value,
            start_deg=self.get_parameter("shovel_start_deg").value,
            pulse_min_us=self.get_parameter("shovel_pulse_min_us").value,
            pulse_max_us=self.get_parameter("shovel_pulse_max_us").value,
            deg_per_tick=1,
        )
        # Los deltas se suman al destino pendiente (igual que "d<n>" en la ESP)
        self.create_subscription(
            Int32, "/servo_delta",
            lambda m: self.shovel_.goto(self.shovel_.target + m.data), 10)
        self.create_subscription(Int32, "/servo_angle",
                                 lambda m: self.shovel_.goto(m.data), 10)
        self.timer_ = self.create_timer(0.02, self.main_loop)  # 50 Hz

        self.motors_ = {
            name: PCAMotor(self.pca_, fwd, bwd)
            for name, (fwd, bwd) in MOTOR_CHANNELS.items()
        }

        self.get_logger().info("Nodo motor_command iniciado (6 motores / 3 puentes H vía PCA9685).")
        self.get_logger().info(
            "Suscrito a /motor_cmd, /armed, /camera_tilt_delta, /camera_tilt_angle, "
            "/servo_delta, /servo_angle")
        self.get_logger().info(
            f"require_arm={self.require_arm_}  cmd_timeout={self.cmd_timeout_} s")
        self.get_logger().info(
            f"PWM a {self.pca_.frequency:.1f} Hz. Cámara en canal "
            f"{self.get_parameter('cam_channel').value}, {self.cam_.angle}°. Pala en canal "
            f"{self.get_parameter('shovel_channel').value}, {self.shovel_.angle}°")
        self.get_logger().info(f"Lado izquierdo: {LEFT_MOTORS}")
        self.get_logger().info(f"Lado derecho:   {RIGHT_MOTORS}")

    def cmd_callback(self, msg: Twist):
        self.latest_cmd_ = msg
        self.last_cmd_time_ = self.get_clock().now()

    def armed_callback(self, msg: Bool):
        self.armed_ = msg.data

    def why_stopped(self):
        """None si se puede mover; si no, el motivo para frenar."""
        if self.require_arm_ and not self.armed_:
            return "desarmado"
        if self.cmd_timeout_ > 0:
            if self.last_cmd_time_ is None:
                return "sin /motor_cmd"
            age = (self.get_clock().now() - self.last_cmd_time_).nanoseconds * 1e-9
            if age > self.cmd_timeout_:
                return f"sin /motor_cmd por más de {self.cmd_timeout_} s"
        return None

    def main_loop(self):
        self.cam_.update()
        self.shovel_.update()

        reason = self.why_stopped()
        if reason != self.stop_reason_:
            if reason:
                self.get_logger().warn(f"Motores frenados: {reason}")
            else:
                self.get_logger().info("Motores habilitados")
            self.stop_reason_ = reason
        if reason:
            for motor in self.motors_.values():
                motor.stop()
            return

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
        self.cam_.release()
        self.shovel_.release()
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
