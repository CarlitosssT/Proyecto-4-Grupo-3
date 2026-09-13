#!/usr/bin/env python3
"""
Nodo ECO — corre en la ESTACIÓN BASE (la laptop).

Va en:  src/vision/vision/link_echo_node.py

Se suscribe al tópico de video comprimido y, por cada frame recibido,
republica SOLO el header (~50 bytes) en un tópico de vuelta. El header
conserva el `stamp` original, puesto por el reloj de la Raspberry.

Por qué esto resuelve el problema del reloj
-------------------------------------------
El rover pone el sello con SU reloj, el sello da la vuelta, y el rover
calcula `ahora - sello` también con SU reloj. Los dos extremos de la resta
vienen del mismo reloj, así que el offset entre máquinas se cancela
exactamente. No necesitas NTP, ni chrony, ni nada.

Lo que obtienes es RTT (round-trip time), no latencia one-way. Y el camino
de vuelta es asimétrico: el video pesa decenas de KB por frame y el eco pesa
50 bytes. Eso significa:

    RTT ≈ latencia_video_ida + latencia_eco_vuelta

y como el eco es minúsculo, la vuelta es casi el tiempo de propagación puro.
En la práctica la ida domina, así que RTT es una COTA SUPERIOR ajustada de
lo que ve el operador. Para decidir una antena eso sobra: te interesa el
punto donde el enlace se degrada, no tres decimales.

Uso
---
    ros2 run vision link_echo_node --ros-args \
        -p topic:=/camera0/image/compressed
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header


def best_effort_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class LinkEchoNode(Node):
    def __init__(self):
        super().__init__("link_echo_node")

        self.declare_parameter("topic", "camera0/image/compressed")
        self.declare_parameter("echo_topic", "")

        topic = self.get_parameter("topic").value
        echo_topic = self.get_parameter("echo_topic").value or "link_echo"

        self.echo_pub_ = self.create_publisher(
            Header, echo_topic, best_effort_qos(depth=10)
        )
        self.subscription_ = self.create_subscription(
            CompressedImage, topic, self.image_callback, best_effort_qos(depth=1)
        )

        self.count_ = 0
        self.get_logger().info(
            f"link_echo_node: {topic}  ->  {echo_topic}  (solo header, ~50 B)"
        )

    def image_callback(self, msg: CompressedImage):
        # El header se reenvía TAL CUAL: mismo stamp, mismo frame_id (que
        # lleva la secuencia). No se toca nada — cualquier modificación
        # contaminaría la medida.
        self.echo_pub_.publish(msg.header)
        self.count_ += 1
        if self.count_ % 240 == 0:
            self.get_logger().info(f"{self.count_} frames eco-devueltos")


def main(args=None):
    rclpy.init(args=args)
    node = LinkEchoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
