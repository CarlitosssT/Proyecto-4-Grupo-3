#!/usr/bin/env python3
"""
Herramienta de excavación (servo de pala + stepper) en la Raspberry Pi.

Port a ROS 2 de la prueba `prueba_servo_stepper.ino` (ESP32):
  - Servo de pala (MG995): ángulo limitado a [angle_min, angle_max].
  - Stepper NEMA 17 con driver A4988 (STEP/DIR), con rampa de aceleración.

Tópicos:
  /stepper_steps  (std_msgs/Int32) pasos RELATIVOS a dar (+/-), se suman al
                  movimiento pendiente (equivale a escribir "200" / "-200").
  /servo_delta    (std_msgs/Int32) grados a sumar al ángulo actual (+/-).
  /servo_angle    (std_msgs/Int32) ir a un ángulo exacto (equivale a "g45").
  /stepper_stop   (std_msgs/Empty) frena el stepper con la aceleración
                  configurada (equivale a "x").

Pines GPIO (BCM) por defecto — no chocan con los de motor_command:
  servo = 12, STEP = 22, DIR = 26. Se pueden cambiar con parámetros.
  EN del A4988 sin conectar (queda activo por defecto).
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int32
from gpiozero import AngularServo, Device, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory(chip=0)

# Rango físico del pulso del servo (MG995 típico: 500 a 2400 us)
SERVO_PULSE_MIN_S = 0.0005
SERVO_PULSE_MAX_S = 0.0024
SERVO_PULSE_SPAN_DEG = 180


class Stepper:
    """STEP/DIR con rampa trapezoidal, ejecutado en un hilo propio."""

    def __init__(self, step_pin, dir_pin, max_speed, acceleration):
        self.step_ = DigitalOutputDevice(step_pin)
        self.dir_ = DigitalOutputDevice(dir_pin)
        self.max_speed_ = float(max_speed)      # pasos/s
        self.accel_ = float(acceleration)       # pasos/s²
        self.lock_ = threading.Lock()
        self.target_ = 0
        self.position_ = 0
        self.speed_ = 0.0
        self.running_ = True
        self.thread_ = threading.Thread(target=self._run, daemon=True)
        self.thread_.start()

    def move(self, steps):
        with self.lock_:
            self.target_ += int(steps)

    def stop(self):
        """Fija el destino lo más cerca posible respetando la rampa de frenado."""
        with self.lock_:
            remaining = self.target_ - self.position_
            sign = 1 if remaining > 0 else -1
            # Pasos necesarios para frenar desde la velocidad actual.
            brake = int(self.speed_ ** 2 / (2.0 * self.accel_)) + 1
            if abs(remaining) > brake:
                self.target_ = self.position_ + sign * brake

    @property
    def position(self):
        return self.position_

    @property
    def distance_to_go(self):
        with self.lock_:
            return self.target_ - self.position_

    def _run(self):
        speed = 0.0
        last = time.perf_counter()
        while self.running_:
            with self.lock_:
                remaining = self.target_ - self.position_
            if remaining == 0 and speed == 0.0:
                time.sleep(0.005)
                last = time.perf_counter()
                continue

            now = time.perf_counter()
            dt = now - last
            last = now

            # ¿Hay que frenar para llegar con velocidad ~0 al destino?
            brake_dist = speed ** 2 / (2.0 * self.accel_)
            if abs(remaining) <= brake_dist or remaining == 0:
                speed = max(speed - self.accel_ * dt, 0.0)
            else:
                speed = min(speed + self.accel_ * dt, self.max_speed_)
            self.speed_ = speed

            if remaining == 0:
                continue
            # Arranque: velocidad mínima para poder dar el primer paso.
            step_speed = max(speed, math.sqrt(2.0 * self.accel_))
            direction = 1 if remaining > 0 else -1

            self.dir_.value = 1 if direction > 0 else 0
            self.step_.on()
            time.sleep(0.00001)
            self.step_.off()
            with self.lock_:
                self.position_ += direction

            time.sleep(1.0 / step_speed)

    def close(self):
        self.running_ = False
        self.thread_.join(timeout=1.0)
        self.step_.close()
        self.dir_.close()


class ToolCommand(Node):
    def __init__(self):
        super().__init__("tool_command")

        self.declare_parameter("servo_pin", 12)
        self.declare_parameter("step_pin", 22)
        self.declare_parameter("dir_pin", 26)
        # Límites de seguridad: el servo NUNCA sale de este rango.
        self.declare_parameter("angle_min", 5)
        self.declare_parameter("angle_max", 130)
        # El servo salta a esta posición al arrancar (pala recogida).
        self.declare_parameter("initial_angle", 30)
        self.declare_parameter("stepper_max_speed", 400.0)   # pasos/s
        self.declare_parameter("stepper_acceleration", 200.0)  # pasos/s²

        p = self.get_parameter
        self.angle_min_ = int(p("angle_min").value)
        self.angle_max_ = int(p("angle_max").value)
        self.angle_ = self._clamp(int(p("initial_angle").value))

        self.servo_ = AngularServo(
            int(p("servo_pin").value),
            min_angle=0,
            max_angle=SERVO_PULSE_SPAN_DEG,
            min_pulse_width=SERVO_PULSE_MIN_S,
            max_pulse_width=SERVO_PULSE_MAX_S,
            initial_angle=self.angle_,
        )
        self.stepper_ = Stepper(
            int(p("step_pin").value),
            int(p("dir_pin").value),
            p("stepper_max_speed").value,
            p("stepper_acceleration").value,
        )

        self.create_subscription(Int32, "/stepper_steps", self.steps_callback, 10)
        self.create_subscription(Empty, "/stepper_stop", self.stop_callback, 10)
        self.create_subscription(Int32, "/servo_delta", self.delta_callback, 10)
        self.create_subscription(Int32, "/servo_angle", self.angle_callback, 10)

        self.get_logger().info(
            f"Nodo tool_command iniciado. Servo en {self.angle_}° "
            f"(límites {self.angle_min_}-{self.angle_max_})."
        )

    def _clamp(self, angle):
        return max(self.angle_min_, min(self.angle_max_, int(angle)))

    def _set_angle(self, angle):
        angle = self._clamp(angle)
        if angle != self.angle_:
            self.angle_ = angle
            self.servo_.angle = angle

    def steps_callback(self, msg: Int32):
        if msg.data != 0:
            self.stepper_.move(msg.data)

    def stop_callback(self, _msg: Empty):
        self.stepper_.stop()
        self.get_logger().info("Stepper: parando...")

    def delta_callback(self, msg: Int32):
        self._set_angle(self.angle_ + msg.data)

    def angle_callback(self, msg: Int32):
        self._set_angle(msg.data)

    def destroy_node(self):
        self.stepper_.close()
        self.servo_.detach()  # deja de hacer fuerza
        self.servo_.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ToolCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
