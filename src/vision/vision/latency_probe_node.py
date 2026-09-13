#!/usr/bin/env python3
"""
Sonda de latencia ONE-WAY — corre en la ESTACIÓN BASE (la laptop).

Va en:  src/vision/vision/latency_probe_node.py

Mide, por cada frame que llega:
  - latencia  = reloj_local_ahora - header.stamp   (ver ADVERTENCIA abajo)
  - jitter    = variación del intervalo entre llegadas
  - pérdida   = huecos en el número de secuencia
  - tamaño    y throughput resultante
  - tiempo de decodificación JPEG (opcional, cuesta CPU)

ADVERTENCIA SOBRE RELOJES — léela antes de creerte los números
---------------------------------------------------------------
`header.stamp` lo puso el reloj de la RASPBERRY. El "ahora" lo pone el reloj
de la LAPTOP. Si los dos relojes difieren en 400 ms, tu latencia medida sale
con 400 ms de sesgo constante — y como el sesgo es constante, parece una
medición perfectamente estable y creíble. Es la trampa clásica.

Tienes dos salidas:

  (a) Sincronizar los relojes con chrony (PTP/NTP en la red local llega a
      ~1 ms, suficiente cuando mides decenas o cientos de ms). Instrucciones
      en GUIA_VISION_LATENCIA.md.

  (b) No sincronizar nada y usar en su lugar el par link_echo_node +
      rtt_probe_node, que mide ida y vuelta usando UN SOLO reloj. Es la
      opción robusta para campo.

Aunque no sincronices, este nodo sigue siendo útil: fps_rx, jitter, pérdida
y throughput NO dependen de la sincronización de relojes, y son justamente
las métricas que deciden la antena.

Uso
---
    ros2 run vision latency_probe_node --ros-args \
        -p topic:=/camera0/image/compressed \
        -p csv_path:=/tmp/rx_d030m.csv \
        -p label:=30m_linea_vista
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np

from vision.link_metrics import (
    parse_frame_id,
    stamp_to_ns,
    RollingStats,
    CsvLogger,
)


def camera_qos_profile() -> QoSProfile:
    # DEBE coincidir con el del publisher. Si el publisher es BEST_EFFORT y
    # el subscriber pide RELIABLE, el QoS es incompatible y NO recibes nada
    # (sin error obvio: simplemente silencio). Compruébalo con:
    #   ros2 topic info /camera0/image/compressed --verbose
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class LatencyProbeNode(Node):
    def __init__(self):
        super().__init__("latency_probe_node")

        self.declare_parameter("topic", "camera0/image/compressed")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("label", "")          # ej. "30m", "50m_pared"
        self.declare_parameter("window_sec", 5.0)
        self.declare_parameter("report_sec", 2.0)
        self.declare_parameter("decode", False)      # medir coste de decode

        topic = self.get_parameter("topic").value
        self.label_ = self.get_parameter("label").value
        self.decode_ = bool(self.get_parameter("decode").value)

        self.stats_ = RollingStats(self.get_parameter("window_sec").value)

        self.csv_ = CsvLogger(
            self.get_parameter("csv_path").value,
            ["label", "seq", "t_rx_ns", "t_tx_ns", "latency_ms",
             "gap_ms", "size_bytes", "decode_ms"],
        )

        self.last_rx_ns_ = None

        self.subscription_ = self.create_subscription(
            CompressedImage, topic, self.image_callback, camera_qos_profile()
        )
        self.timer_ = self.create_timer(
            self.get_parameter("report_sec").value, self.report
        )

        self.get_logger().info(
            f"latency_probe_node escuchando en {topic}"
            + (f"  label={self.label_}" if self.label_ else "")
            + (f"  csv={self.csv_.path}" if self.csv_.enabled else "  (sin CSV)")
        )
        self.get_logger().warn(
            "La latencia one-way asume relojes sincronizados entre rover y base. "
            "Si no lo están, fíjate en fps_rx / jitter / loss, que no dependen del reloj."
        )

    # ----------------------------------------------------------------------
    def image_callback(self, msg: CompressedImage):
        t_rx_ns = self.get_clock().now().nanoseconds
        t_tx_ns = stamp_to_ns(msg.header.stamp)
        _, seq = parse_frame_id(msg.header.frame_id)

        latency_ms = (t_rx_ns - t_tx_ns) / 1e6
        gap_ms = (
            (t_rx_ns - self.last_rx_ns_) / 1e6
            if self.last_rx_ns_ is not None else float("nan")
        )
        self.last_rx_ns_ = t_rx_ns

        decode_ms = float("nan")
        if self.decode_:
            t0 = time.perf_counter()
            cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            decode_ms = (time.perf_counter() - t0) * 1000.0

        self.stats_.add(t_rx_ns, latency_ms, len(msg.data), seq)

        self.csv_.write({
            "label": self.label_,
            "seq": seq if seq is not None else "",
            "t_rx_ns": t_rx_ns,
            "t_tx_ns": t_tx_ns,
            "latency_ms": round(latency_ms, 3),
            "gap_ms": round(gap_ms, 3) if gap_ms == gap_ms else "",
            "size_bytes": len(msg.data),
            "decode_ms": round(decode_ms, 3) if decode_ms == decode_ms else "",
        })

    # ----------------------------------------------------------------------
    def report(self):
        s = self.stats_.summary()
        prefix = f"[{self.label_}] " if self.label_ else ""
        self.get_logger().info(prefix + RollingStats.format(s))

    def destroy_node(self):
        s = self.stats_.summary()
        if s.get("n", 0):
            self.get_logger().info(
                f"TOTAL recibidos={self.stats_.total_rx}  "
                f"perdidos={self.stats_.total_lost}"
            )
        self.csv_.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LatencyProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
