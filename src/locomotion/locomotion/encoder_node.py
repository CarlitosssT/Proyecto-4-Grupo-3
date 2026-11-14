#!/usr/bin/env python3
"""
Encoders de las 6 ruedas: convierte los conteos crudos en ángulo y velocidad.

La ESP32 DevKit cuenta los pulsos (firmware/tool_esp32/tool_esp32.ino) y
tool_command, que es el dueño del puerto serial, publica los conteos tal cual
en /encoder_ticks. Este nodo NO abre el serial: solo hace la cuenta. (Dos
programas no pueden leer el mismo puerto: se repartirían las líneas al azar.)

Entrada:
  /encoder_ticks (std_msgs/Int64MultiArray)
      data[0]    = micros() de la ESP al leer los contadores (da la vuelta
                   cada ~71 min)
      data[1..6] = conteos en el orden de WHEELS (enteros de 32 bits, en
                   cuadratura x4: 4 cuentas por pulso del encoder)

Salida:
  /wheel_states (sensor_msgs/JointState)
      name     = WHEELS
      position = ángulo acumulado de cada rueda [rad]
      velocity = velocidad de cada rueda [rad/s]

Parámetros:
  counts_per_rev  Cuentas por VUELTA DE RUEDA = pulsos por vuelta del encoder
                  x 4 x reducción. Con 0 el nodo entra en modo calibración: no
                  publica /wheel_states y muestra los conteos para medirlo.
  invert          6 booleanos en el orden de WHEELS. True le cambia el signo a
                  esa rueda (los motores de un lado van montados en espejo).

Una vez medidos, se pueden dejar fijos en DEFAULT_COUNTS_PER_REV y
DEFAULT_INVERT para no pasarlos cada vez.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray

# Mismo orden que las filas de PIN_ENC en el firmware
WHEELS = (
    "front_left", "mid_left", "rear_left",
    "front_right", "mid_right", "rear_right",
)

DEFAULT_COUNTS_PER_REV = 0.0            # 0 = sin calibrar
DEFAULT_INVERT = [False] * len(WHEELS)

# Más que esto entre dos reportes seguidos = se perdió el hilo (la ESP se
# reinició o se desconectó): no se calcula velocidad con ese par.
MAX_GAP_US = 1_000_000


def wrap32(value):
    """Diferencia de dos enteros de 32 bits, aunque el contador haya dado la vuelta."""
    return ((value + 2**31) % 2**32) - 2**31


class EncoderNode(Node):
    def __init__(self):
        super().__init__("encoder_node")

        self.declare_parameter("counts_per_rev", DEFAULT_COUNTS_PER_REV)
        self.declare_parameter("invert", DEFAULT_INVERT)

        self.cpr_ = float(self.get_parameter("counts_per_rev").value)
        invert = list(self.get_parameter("invert").value)
        if len(invert) != len(WHEELS):
            raise ValueError(
                f"invert debe tener {len(WHEELS)} valores (uno por rueda), tiene {len(invert)}")
        self.sign_ = [-1 if inv else 1 for inv in invert]

        self.prev_t_ = None
        self.prev_counts_ = None
        self.total_ = [0] * len(WHEELS)     # cuentas acumuladas, ya con signo
        self.shown_ = None                  # último total mostrado (calibración)

        self.pub_ = self.create_publisher(JointState, "/wheel_states", 10)
        self.create_subscription(Int64MultiArray, "/encoder_ticks", self.ticks_callback, 10)

        if self.cpr_ > 0:
            self.get_logger().info(
                f"encoder_node listo: {self.cpr_:g} cuentas por vuelta de rueda. "
                "Publica /wheel_states (rad y rad/s).")
        else:
            self.create_timer(1.0, self.show_counts)
            self.get_logger().warn(
                "MODO CALIBRACIÓN (falta counts_per_rev): no se publica /wheel_states.\n"
                "  1. Marca una rueda y dale 10 vueltas a mano.\n"
                "  2. Divide entre 10 lo que cambió su conteo (se muestra abajo).\n"
                "  3. Relanza con: --ros-args -p counts_per_rev:=<ese valor>\n"
                "  Si una rueda cuenta negativo al avanzar, ponla en True en 'invert'.")

    def ticks_callback(self, msg: Int64MultiArray):
        if len(msg.data) != 1 + len(WHEELS):
            self.get_logger().warn(
                f"/encoder_ticks con {len(msg.data)} valores (se esperaban {1 + len(WHEELS)}).",
                throttle_duration_sec=5.0)
            return

        t_us, counts = msg.data[0], list(msg.data[1:])
        if self.prev_t_ is None:
            self.prev_t_, self.prev_counts_ = t_us, counts
            return

        dt_us = (t_us - self.prev_t_) % 2**32
        delta = [wrap32(c - p) * s
                 for c, p, s in zip(counts, self.prev_counts_, self.sign_)]
        self.prev_t_, self.prev_counts_ = t_us, counts

        if dt_us == 0 or dt_us > MAX_GAP_US:
            self.get_logger().warn(
                "Salto en los reportes de la ESP (¿se reinició?): se retoma el conteo.")
            return

        for i, d in enumerate(delta):
            self.total_[i] += d
        if self.cpr_ <= 0:
            return

        rad_per_count = 2.0 * math.pi / self.cpr_
        dt = dt_us * 1e-6
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(WHEELS)
        out.position = [n * rad_per_count for n in self.total_]
        out.velocity = [d * rad_per_count / dt for d in delta]
        self.pub_.publish(out)

    def show_counts(self):
        """Modo calibración: muestra los conteos cuando cambian."""
        if self.prev_t_ is None:
            self.get_logger().info(
                "Sin datos en /encoder_ticks (¿está corriendo tool_command?)",
                throttle_duration_sec=5.0)
            return
        if self.total_ == self.shown_:
            return
        self.shown_ = list(self.total_)
        self.get_logger().info(
            "conteos: " + "  ".join(f"{w}={n}" for w, n in zip(WHEELS, self.total_)))


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
